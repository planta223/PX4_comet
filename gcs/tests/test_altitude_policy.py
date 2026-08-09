import math
import unittest

import fusion
import sik


def lora_sample(alt=10.0, position_valid=True, altitude_valid=True):
    return {
        "pos_e": 1.0,
        "pos_n": 2.0,
        "position_valid": position_valid,
        "alt": alt,
        "altitude_valid": altitude_valid,
        "timestamp": 1.0,
    }


def update(data_fusion, *, sik_alive, lora_alive, sik_alt=20.0, sample=None):
    return data_fusion.update(
        1.0,
        sik_alive,
        lora_alive,
        (100.0, 200.0, sik_alt),
        (0.0, 0.0, 0.0),
        None,
        (0.0, 0.0, 0.0),
        None,
        sample,
        "ASCENT" if sample is not None else None,
    )


class AltitudePolicyTests(unittest.TestCase):
    def test_lora_altitude_wins_when_both_links_alive(self):
        result = update(
            fusion.DataFusion(), sik_alive=True, lora_alive=True,
            sample=lora_sample(12.5),
        )
        self.assertEqual(result["position"][2], 12.5)
        self.assertEqual(result["altitude_source"], "lora_baro")

    def test_sik_is_explicit_fallback_when_lora_dead(self):
        result = update(fusion.DataFusion(), sik_alive=True, lora_alive=False, sik_alt=21.0)
        self.assertEqual(result["position"][2], 21.0)
        self.assertEqual(result["altitude_source"], "sik_fallback")

    def test_both_dead_freezes_last_valid_altitude(self):
        data_fusion = fusion.DataFusion()
        update(data_fusion, sik_alive=True, lora_alive=True, sample=lora_sample(33.0))
        result = update(data_fusion, sik_alive=False, lora_alive=False)
        self.assertEqual(result["position"][2], 33.0)
        self.assertEqual(result["altitude_source"], "lora_baro")

    def test_invalid_lora_altitude_falls_back_to_sik(self):
        for invalid in (math.nan, math.inf, -math.inf, None):
            with self.subTest(invalid=invalid):
                result = update(
                    fusion.DataFusion(), sik_alive=True, lora_alive=True,
                    sik_alt=44.0, sample=lora_sample(invalid),
                )
                self.assertEqual(result["position"][2], 44.0)
                self.assertEqual(result["altitude_source"], "sik_fallback")

    def test_horizontal_position_and_altitude_sources_are_independent(self):
        result = update(
            fusion.DataFusion(), sik_alive=True, lora_alive=True,
            sample=lora_sample(55.0, position_valid=False),
        )
        self.assertEqual(result["position"][:2], (100.0, 200.0))
        self.assertEqual(result["position_source"], "sik")
        self.assertEqual(result["position"][2], 55.0)
        self.assertEqual(result["altitude_source"], "lora_baro")

    def test_source_changes_without_offset_or_smoothing(self):
        data_fusion = fusion.DataFusion()
        first = update(data_fusion, sik_alive=True, lora_alive=True, sample=lora_sample(10.0))
        second = update(data_fusion, sik_alive=True, lora_alive=False, sik_alt=30.0)
        self.assertEqual(first["position"][2], 10.0)
        self.assertEqual(second["position"][2], 30.0)
        self.assertEqual(second["altitude_source"], "sik_fallback")

    def test_sik_baro_origin_survives_link_reconnect_but_resets_on_fc_reboot(self):
        tracker = sik._BaroOriginTracker()
        tracker.set_if_needed(1000.0, 10_000)
        self.assertAlmostEqual(tracker.to_relative_altitude(990.0), 84.69, places=1)

        tracker.set_if_needed(990.0, 20_000)  # same FC after link reconnect
        self.assertEqual(tracker.press0_hpa, 1000.0)

        tracker.set_if_needed(980.0, 100)  # FC time_boot_ms restarted
        self.assertEqual(tracker.press0_hpa, 980.0)
        self.assertEqual(tracker.to_relative_altitude(980.0), 0.0)


if __name__ == "__main__":
    unittest.main()
