# -*- coding: utf-8 -*-
"""
비행 사후분석 CLI.

main.py가 남긴 logs/flight_<타임스탬프>/ 폴더 하나를 읽어 report.html
하나로 요약한다. main.py(실전용)는 CSV/JSONL만 남기고 절대 인터넷/무거운
렌더링을 하지 않고, 이 스크립트가 그 데이터를 나중에(착륙 후, 어느
컴퓨터에서든) 소비하는 역할을 전담한다.

개발단계 우선순위(실비행 전이라 파이프라인/시스템 진단이 궤적/개요보다
중요하다는 판단)로 섹션을 배치했다:
  1순위: 리부트 검증 결과, 링크 품질, SiK-LoRa 위치 일치도
  2순위: 상태 타임라인 + 명령 이력, 경고/에러 메시지
  3순위: 궤적(로컬 ENU 평면도, 지도 오버레이는 후속 작업), 개요, 설정 스냅샷

결과(report.html)는 이미지까지 base64로 파일 하나에 다 담아서, 팀원에게
공유할 때 이 파일 하나만 보내면 된다 (별도 소프트웨어 설치 불필요, 브라우저면 충분).

사용법: python analyze_flight.py logs/flight_20260724_153000
"""

import argparse
import base64
import csv
import glob
import html
import io
import json
import math
import os
import statistics
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# matplotlib 기본 폰트(DejaVu Sans)엔 한글 글리프가 없어 깨진다 - main.py가
# dearpygui에 맑은 고딕을 쓰는 것과 동일한 이유/폰트로 맞춘다.
plt.rcParams["font.family"] = "Malgun Gothic"
plt.rcParams["axes.unicode_minus"] = False

# main.py의 _MAV_SEVERITY_COLORS와 동일한 기준 - 0~4(EMERGENCY~WARNING)만
# "주목해야 할 메시지"로 뽑는다. NOTICE/INFO/DEBUG(5~7)는 리포트에서 제외.
_WARNING_OR_WORSE_SEVERITY = 4


# ---------------------------------------------------------------------------
# 파일 로딩
# ---------------------------------------------------------------------------

def _load_json(path):
    if not path or not os.path.exists(path):
        return None
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def _load_jsonl(path):
    if not path or not os.path.exists(path):
        return []
    records = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                records.append(json.loads(line))
    return records


def _load_csv_rows(path):
    if not path or not os.path.exists(path):
        return []
    with open(path, encoding="utf-8") as f:
        return list(csv.DictReader(f))


def _find_one(flight_dir, pattern):
    matches = sorted(glob.glob(os.path.join(flight_dir, pattern)))
    return matches[0] if matches else None


def _to_float(v):
    try:
        if v is None or v == "":
            return None
        return float(v)
    except (TypeError, ValueError):
        return None


def load_flight_data(flight_dir):
    return {
        "flight_dir": flight_dir,
        "manifest": _load_json(os.path.join(flight_dir, "manifest.json")),
        "events": _load_jsonl(os.path.join(flight_dir, "events.jsonl")),
        "sik_rows": _load_csv_rows(_find_one(flight_dir, "sik_*.csv")),
        "sik_stats_rows": _load_csv_rows(_find_one(flight_dir, "sik_stats_*.csv")),
        "lora_rows": _load_csv_rows(_find_one(flight_dir, "lora_*.csv")),
        "lora_stats_rows": _load_csv_rows(_find_one(flight_dir, "lora_stats_*.csv")),
        "fusion_compare_rows": _load_csv_rows(_find_one(flight_dir, "fusion_compare_stats_*.csv")),
    }


# ---------------------------------------------------------------------------
# 섹션별 가공 (1순위: 리부트 검증/링크 품질/SiK-LoRa 일치도)
# ---------------------------------------------------------------------------

def section_reboot_verify(manifest):
    if manifest is None:
        return None
    return manifest.get("reboot_verify")


def _summarize_link_stats(rows, rate_key, loss_key, extra_key=None):
    if not rows:
        return None
    rates = [v for v in (_to_float(r.get(rate_key)) for r in rows) if v is not None]
    jitters = [v for v in (_to_float(r.get("jitter_ms")) for r in rows) if v is not None]
    losses = [v for v in (_to_float(r.get(loss_key)) for r in rows) if v is not None]
    result = {
        "rate_mean": statistics.mean(rates) if rates else None,
        "jitter_mean": statistics.mean(jitters) if jitters else None,
        "jitter_max": max(jitters) if jitters else None,
        "loss_mean": statistics.mean(losses) if losses else None,
        "loss_max": max(losses) if losses else None,
    }
    if extra_key:
        extras = [v for v in (_to_float(r.get(extra_key)) for r in rows) if v is not None]
        result["crc_fail_total"] = sum(extras) if extras else None
    return result


def section_link_quality(sik_stats_rows, lora_stats_rows):
    return {
        "sik": _summarize_link_stats(sik_stats_rows, "rate_hz", "link_loss_pct"),
        "lora": _summarize_link_stats(lora_stats_rows, "rate_hz", "seq_loss_pct", "crc_fail_count"),
    }


def section_fusion_compare(rows):
    if not rows:
        return None
    fields = ["diff_mean_e", "diff_mean_n", "diff_mean_alt", "diff_std_e", "diff_std_n", "diff_std_alt"]
    last = rows[-1]
    return {f: _to_float(last.get(f)) for f in fields}


# ---------------------------------------------------------------------------
# 섹션별 가공 (2순위: 타임라인/명령이력/경고메시지)
# ---------------------------------------------------------------------------

def section_state_timeline(events):
    transitions = [e for e in events if e.get("type") == "rocket_state_transition"]
    if not transitions:
        return []
    t0 = transitions[0]["ts"]
    return [{**e, "t_plus": e["ts"] - t0} for e in transitions]


_COMMAND_EVENT_TYPES = {
    "servo_command", "lora_command_result", "reboot_requested", "reboot_verify_result",
    "reboot_verify_servo_lock_sent", "gcs_ready_authorized", "gcs_ready_authorization_revoked",
}


def section_command_history(events):
    return [e for e in events if e.get("type") in _COMMAND_EVENT_TYPES]


def section_messages(events):
    return [
        e for e in events
        if e.get("type") == "message" and e.get("severity", 99) <= _WARNING_OR_WORSE_SEVERITY
    ]


# ---------------------------------------------------------------------------
# 섹션별 가공 (3순위: 개요/궤적/설정 스냅샷)
# ---------------------------------------------------------------------------

def section_overview(sik_rows):
    alts = [v for v in (_to_float(r.get("kf_alt")) for r in sik_rows) if v is not None]
    speeds = []
    for r in sik_rows:
        ve, vn, vz = _to_float(r.get("kf_vel_e")), _to_float(r.get("kf_vel_n")), _to_float(r.get("kf_vel_z"))
        if None not in (ve, vn, vz):
            speeds.append(math.sqrt(ve * ve + vn * vn + vz * vz))

    recovery = None
    for r in reversed(sik_rows):
        lat, lon = _to_float(r.get("raw_lat")), _to_float(r.get("raw_lon"))
        if lat is not None and lon is not None:
            recovery = (lat, lon)
            break

    duration_sec = None
    if sik_rows:
        first_ts, last_ts = _to_float(sik_rows[0].get("timestamp")), _to_float(sik_rows[-1].get("timestamp"))
        if first_ts is not None and last_ts is not None:
            duration_sec = last_ts - first_ts

    return {
        "max_alt_m": max(alts) if alts else None,
        "max_speed_mps": max(speeds) if speeds else None,
        "recovery_lat_lon": recovery,
        "duration_sec": duration_sec,
    }


def plot_trajectory_png_base64(sik_rows, lora_rows):
    """
    로컬 ENU 평면도 (v1) - 인터넷 지도 타일 오버레이는 안 한다. main.py와
    마찬가지로 이 스크립트도 실행 환경에 인터넷이 없을 수 있다는 전제라,
    항상 그릴 수 있는 오프라인 방식을 기본으로 하고 지도 배경은 후속 작업.
    """
    fig, ax = plt.subplots(figsize=(6, 6))
    has_any = False

    sik_e = [v for v in (_to_float(r.get("kf_pos_e")) for r in sik_rows) if v is not None]
    sik_n = [v for v in (_to_float(r.get("kf_pos_n")) for r in sik_rows) if v is not None]
    if sik_e and sik_n:
        ax.plot(sik_e, sik_n, color="green", linewidth=1.5, label="SiK")
        has_any = True

    if lora_rows:
        # comet_sensor.lua가 HOME 기준 로컬 좌표(pos_n/pos_e, m)로 이미 보내주므로
        # (lora.py 참고) origin/위경도 변환이 필요 없다 - CSV 값을 그대로 쓴다.
        lora_e, lora_n = [], []
        for r in lora_rows:
            pos_n, pos_e = _to_float(r.get("pos_n")), _to_float(r.get("pos_e"))
            if pos_n is not None and pos_e is not None:
                lora_e.append(pos_e)
                lora_n.append(pos_n)
        if lora_e:
            ax.plot(lora_e, lora_n, color="magenta", linewidth=1.2, linestyle="--", label="LoRa")
            has_any = True

    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_title("궤적 (로컬 ENU 평면도)")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    if has_any:
        ax.legend()
    else:
        ax.text(0.5, 0.5, "궤적 데이터 없음", ha="center", va="center", transform=ax.transAxes)

    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=120, bbox_inches="tight")
    plt.close(fig)
    buf.seek(0)
    return base64.b64encode(buf.read()).decode("ascii")


# ---------------------------------------------------------------------------
# HTML 렌더링
# ---------------------------------------------------------------------------

def _fmt(value, unit="", digits=2):
    if value is None:
        return "-"
    if isinstance(value, float):
        return f"{value:.{digits}f}{unit}"
    return f"{value}{unit}"


def _esc(text):
    return html.escape(str(text))


def _render_reboot_verify(section):
    if not section:
        return "<p>리부트 검증 데이터 없음 (인가 없이 만들어진 폴더이거나 manifest.json 누락)</p>"
    rows = "".join(
        f"<tr><td>{_esc(name)}</td><td class='{'ok' if passed else 'fail'}'>{'통과' if passed else '실패/대기'}</td></tr>"
        for name, passed in section.items()
    )
    return f"<table><tr><th>항목</th><th>결과</th></tr>{rows}</table>"


def _render_link_quality(section):
    def row(label, s):
        if s is None:
            return f"<tr><td>{label}</td><td colspan='5'>데이터 없음</td></tr>"
        extra = _fmt(s.get("crc_fail_total"), digits=0) if "crc_fail_total" in s else "-"
        return (
            f"<tr><td>{label}</td><td>{_fmt(s['rate_mean'], ' Hz')}</td>"
            f"<td>{_fmt(s['jitter_mean'], ' ms')}</td><td>{_fmt(s['jitter_max'], ' ms')}</td>"
            f"<td>{_fmt(s['loss_mean'], ' %')}</td><td>{_fmt(s['loss_max'], ' %')}</td><td>{extra}</td></tr>"
        )

    return (
        "<table><tr><th>링크</th><th>평균 rate</th><th>평균 jitter</th><th>최대 jitter</th>"
        "<th>평균 손실률</th><th>최대 손실률</th><th>CRC 실패 누적</th></tr>"
        + row("SiK", section["sik"]) + row("LoRa", section["lora"]) + "</table>"
    )


def _render_fusion_compare(section):
    if section is None:
        return "<p>SiK/LoRa가 동시에 살아있던 구간이 없어 비교 데이터 없음</p>"
    return (
        "<table><tr><th>축</th><th>평균 차(SiK-LoRa)</th><th>표준편차</th></tr>"
        f"<tr><td>East</td><td>{_fmt(section['diff_mean_e'], ' m')}</td><td>{_fmt(section['diff_std_e'], ' m')}</td></tr>"
        f"<tr><td>North</td><td>{_fmt(section['diff_mean_n'], ' m')}</td><td>{_fmt(section['diff_std_n'], ' m')}</td></tr>"
        f"<tr><td>Alt</td><td>{_fmt(section['diff_mean_alt'], ' m')}</td><td>{_fmt(section['diff_std_alt'], ' m')}</td></tr>"
        "</table>"
    )


def _render_state_timeline(transitions):
    if not transitions:
        return "<p>상태 전이 기록 없음</p>"
    rows = "".join(
        f"<tr><td>T+{e['t_plus']:.1f}s</td><td>{_esc(e.get('from_state'))} -&gt; {_esc(e.get('to_state'))}</td></tr>"
        for e in transitions
    )
    return f"<table><tr><th>시각</th><th>전이</th></tr>{rows}</table>"


def _render_command_history(events):
    if not events:
        return "<p>명령/이벤트 기록 없음</p>"
    rows = "".join(
        f"<tr><td>{_esc(e.get('type'))}</td>"
        f"<td>{_esc({k: v for k, v in e.items() if k not in ('ts', 'type')})}</td></tr>"
        for e in events
    )
    return f"<table><tr><th>종류</th><th>세부내용</th></tr>{rows}</table>"


def _render_messages(events):
    if not events:
        return "<p>경고/에러 메시지 없음</p>"
    rows = "".join(
        f"<tr><td>{e.get('severity')}</td><td>{_esc(e.get('text'))}</td></tr>"
        for e in events
    )
    return f"<table><tr><th>severity</th><th>메시지</th></tr>{rows}</table>"


def _render_overview(section):
    lat_lon = section["recovery_lat_lon"]
    recovery_text = f"{lat_lon[0]:.6f}, {lat_lon[1]:.6f}" if lat_lon else "-"
    return (
        "<table>"
        f"<tr><td>비행 시간</td><td>{_fmt(section['duration_sec'], ' s')}</td></tr>"
        f"<tr><td>최고 고도</td><td>{_fmt(section['max_alt_m'], ' m')}</td></tr>"
        f"<tr><td>최고 속도</td><td>{_fmt(section['max_speed_mps'], ' m/s')}</td></tr>"
        f"<tr><td>회수 좌표</td><td>{recovery_text}</td></tr>"
        "</table>"
    )


def _render_manifest(manifest):
    if manifest is None:
        return "<p>manifest.json 없음</p>"
    pretty = json.dumps(manifest, ensure_ascii=False, indent=2)
    return f"<pre>{_esc(pretty)}</pre>"


_CSS = """
body { font-family: -apple-system, "Malgun Gothic", sans-serif; margin: 2rem; color: #1a1a1a; }
h1 { margin-bottom: 0.2rem; }
.subtitle { color: #666; margin-top: 0; }
h2 { border-bottom: 2px solid #ddd; padding-bottom: 0.3rem; margin-top: 2.5rem; }
table { border-collapse: collapse; width: 100%; margin: 0.5rem 0; }
th, td { border: 1px solid #ddd; padding: 0.4rem 0.6rem; text-align: left; font-size: 0.9rem; }
th { background: #f5f5f5; }
td.ok { color: #1a7f37; font-weight: bold; }
td.fail { color: #cf222e; font-weight: bold; }
pre { background: #f5f5f5; padding: 1rem; overflow-x: auto; font-size: 0.85rem; }
img { max-width: 100%; }
.priority-tag { font-size: 0.7rem; background: #eee; border-radius: 4px; padding: 0.1rem 0.4rem; margin-left: 0.5rem; color: #666; }
"""


def render_report_html(data):
    manifest = data["manifest"]
    events = data["events"]

    reboot_verify = section_reboot_verify(manifest)
    link_quality = section_link_quality(data["sik_stats_rows"], data["lora_stats_rows"])
    fusion_compare = section_fusion_compare(data["fusion_compare_rows"])
    state_timeline = section_state_timeline(events)
    command_history = section_command_history(events)
    messages = section_messages(events)
    overview = section_overview(data["sik_rows"])
    trajectory_png_b64 = plot_trajectory_png_base64(data["sik_rows"], data["lora_rows"])

    flight_name = os.path.basename(os.path.normpath(data["flight_dir"]))

    return f"""<!doctype html>
<html lang="ko"><head><meta charset="utf-8"><title>비행 분석 리포트 - {_esc(flight_name)}</title>
<style>{_CSS}</style></head>
<body>
<h1>비행 분석 리포트</h1>
<p class="subtitle">{_esc(data['flight_dir'])}</p>

<h2>리부트 검증 결과 <span class="priority-tag">1순위</span></h2>
{_render_reboot_verify(reboot_verify)}

<h2>링크 품질 <span class="priority-tag">1순위</span></h2>
{_render_link_quality(link_quality)}

<h2>SiK-LoRa 위치 일치도 <span class="priority-tag">1순위</span></h2>
{_render_fusion_compare(fusion_compare)}

<h2>상태 타임라인 <span class="priority-tag">2순위</span></h2>
{_render_state_timeline(state_timeline)}

<h2>명령/이벤트 이력 <span class="priority-tag">2순위</span></h2>
{_render_command_history(command_history)}

<h2>경고/에러 메시지 <span class="priority-tag">2순위</span></h2>
{_render_messages(messages)}

<h2>개요 <span class="priority-tag">3순위</span></h2>
{_render_overview(overview)}

<h2>궤적 (로컬 ENU) <span class="priority-tag">3순위</span></h2>
<img src="data:image/png;base64,{trajectory_png_b64}" alt="trajectory">

<h2>설정 스냅샷 <span class="priority-tag">3순위</span></h2>
{_render_manifest(manifest)}

</body></html>
"""


def main():
    parser = argparse.ArgumentParser(description="비행 로그 폴더를 report.html로 요약한다.")
    parser.add_argument("flight_dir", help="예: logs/flight_20260724_153000")
    args = parser.parse_args()

    if not os.path.isdir(args.flight_dir):
        print(f"디렉터리를 찾을 수 없음: {args.flight_dir}", file=sys.stderr)
        sys.exit(1)

    data = load_flight_data(args.flight_dir)
    report_html = render_report_html(data)

    output_path = os.path.join(args.flight_dir, "report.html")
    with open(output_path, "w", encoding="utf-8") as f:
        f.write(report_html)

    print(f"리포트 생성됨: {output_path}")


if __name__ == "__main__":
    main()
