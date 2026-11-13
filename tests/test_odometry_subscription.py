"""Wheel odometry keeps listening (field 2026-11-13: the whole stream was
wrapped in a 5 s timeout, so a working subscription was cancelled after 5 s
and every run integrated the commanded speed instead of the measured one)."""
from __future__ import annotations

import asyncio
import sys
import time
import types

import navigation.odometry as odo_mod


class _Msg:
    def __init__(self, v, w):
        self.measured_speed, self.measured_angular_rate = v, w


def _install_fake_sdk(monkeypatch, paths_ok):
    class FakeClient:
        def __init__(self, cfg):
            pass

        def subscribe(self, req, decode=True):
            path = req.uri.path

            async def gen():
                if path not in paths_ok:
                    await asyncio.sleep(10)          # never answers → timeout
                k = 0
                while True:
                    k += 1
                    yield None, _Msg(0.2, 0.01 * k)
                    await asyncio.sleep(0.01)
            return gen()

    class Uri:
        def __init__(self, path="", query=""):
            self.path = path

    class SubscribeRequest:
        def __init__(self, uri=None, every_n=1):
            self.uri = uri

    mods = {
        "farm_ng": types.ModuleType("farm_ng"),
        "farm_ng.core": types.ModuleType("farm_ng.core"),
        "farm_ng.core.event_client": types.ModuleType("farm_ng.core.event_client"),
        "farm_ng.core.event_service_pb2": types.ModuleType("farm_ng.core.event_service_pb2"),
        "farm_ng.core.uri_pb2": types.ModuleType("farm_ng.core.uri_pb2"),
    }
    mods["farm_ng.core.event_client"].EventClient = FakeClient
    mods["farm_ng.core.event_service_pb2"].SubscribeRequest = SubscribeRequest
    mods["farm_ng.core.uri_pb2"].Uri = Uri
    for k, m in mods.items():
        monkeypatch.setitem(sys.modules, k, m)


def _run_for(od, secs):
    async def main():
        task = asyncio.ensure_future(od.run())
        await asyncio.sleep(secs)
        task.cancel()
        try:
            await task
        except BaseException:
            pass
    asyncio.run(main())


def test_subscription_survives_past_the_first_message_timeout(monkeypatch):
    monkeypatch.setattr(odo_mod, "_SUBSCRIBE_TIMEOUT", 0.2)
    _install_fake_sdk(monkeypatch, paths_ok={"/state"})
    od = odo_mod.WheelOdometry(canbus_config=object())
    _run_for(od, 0.8)                                # 4× the timeout
    assert od.available
    assert time.monotonic() - od._meas_t < 0.1       # still receiving at the end
    assert od._meas_angular > 0.3                    # many messages, not one
    od.tick(0.0, 0.0, 0.1)
    assert od.last_source == "wheel" and od.last_speed == 0.2


def test_falls_through_to_a_later_path(monkeypatch):
    monkeypatch.setattr(odo_mod, "_SUBSCRIBE_TIMEOUT", 0.1)
    _install_fake_sdk(monkeypatch, paths_ok={"/canbus"})
    od = odo_mod.WheelOdometry(canbus_config=object())
    _run_for(od, 0.6)
    assert od.available and time.monotonic() - od._meas_t < 0.1


def test_no_stream_falls_back_to_commanded(monkeypatch, capsys):
    monkeypatch.setattr(odo_mod, "_SUBSCRIBE_TIMEOUT", 0.05)
    _install_fake_sdk(monkeypatch, paths_ok=set())
    od = odo_mod.WheelOdometry(canbus_config=object())
    _run_for(od, 0.4)
    assert not od.available
    assert "no wheel-speed stream" in capsys.readouterr().out
    od.tick(0.15, 0.0, 0.1)
    assert od.last_source == "cmd"
