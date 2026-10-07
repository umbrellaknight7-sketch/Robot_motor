from linker_hand_l30_v6.wrist_mux import WristMux


def test_disjoint_commands_and_fencing():
    mux = WristMux()
    assert not mux.fingers([1.] * 17)
    assert not mux.arm(0.)
    assert mux.feedback([2.] * 17, 0.)
    token = mux.arm(0.)
    assert mux.wrist(token, -30., .1)
    assert mux.fingers([8.] * 17)
    assert mux.snapshot(.1) == [8.] * 16 + [-30.]
    mux.stop()
    assert not mux.wrist(token, 5., .2)
    assert mux.snapshot(.2) == [8.] * 16 + [2.]
    assert mux.arm(.2) != token


def test_timeout_is_wrist_only_and_latched():
    mux = WristMux()
    mux.feedback([0.] * 17, 0.)
    token = mux.arm(0.)
    mux.wrist(token, 30., .1)
    assert not mux.wrist(token, float('nan'), .4)
    assert not mux.wrist(token, 10000., .4)
    mux.feedback([5.] * 17, .6)
    mux.check(.61)
    assert mux.fault and not mux.token
    assert mux.fingers([7.] * 16)
    assert mux.snapshot(.61) == [7.] * 16 + [5.]
    assert not mux.arm(.61)
    assert mux.reset(.62)
    assert mux.arm(.62)
    assert not mux.reset(2.)


def test_invalid_feedback_does_not_refresh_watchdog():
    mux = WristMux()
    mux.feedback([0.] * 17, 0.)
    token = mux.arm(0.)
    assert not mux.feedback([float('nan')] * 17, .4)
    assert not mux.wrist(token, 1., .6)
    assert mux.fault
