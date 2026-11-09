import pytest

from mav.io.buttons import Event, GpioButtons


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


class FakeGPIO:
    BOARD = object()
    IN = object()
    PUD_UP = object()
    HIGH = 1
    LOW = 0

    def __init__(self):
        self.states = {}
        self.setup_calls = []
        self.cleanup_calls = []

    def setmode(self, mode):
        assert mode is self.BOARD

    def setup(self, pin, mode, *, pull_up_down):
        assert mode is self.IN
        assert pull_up_down is self.PUD_UP
        self.setup_calls.append(pin)
        self.states.setdefault(pin, self.HIGH)

    def input(self, pin):
        return self.states[pin]

    def cleanup(self, pins):
        self.cleanup_calls.append(list(pins))


def test_gpio_buttons_debounce_one_press_and_rearm_on_release():
    gpio = FakeGPIO()
    clock = FakeClock()
    buttons = GpioButtons(11, 13, debounce_s=0.05, gpio=gpio, clock=clock)

    assert buttons.poll() == []
    gpio.states[11] = gpio.LOW
    assert buttons.poll() == []
    clock.advance(0.049)
    assert buttons.poll() == []
    clock.advance(0.001)
    assert buttons.poll() == [Event.START]
    assert buttons.poll() == []
    clock.advance(0.1)
    gpio.states[11] = gpio.HIGH
    assert buttons.poll() == []
    clock.advance(0.05)
    assert buttons.poll() == []
    gpio.states[11] = gpio.LOW
    clock.advance(0.05)
    assert buttons.poll() == [Event.START]


def test_gpio_buttons_reports_both_buttons_without_blocking():
    gpio = FakeGPIO()
    clock = FakeClock()
    buttons = GpioButtons(11, 13, debounce_s=0.05, gpio=gpio, clock=clock)

    gpio.states[11] = gpio.LOW
    gpio.states[13] = gpio.LOW
    assert buttons.poll() == []
    clock.advance(0.05)
    assert buttons.poll() == [Event.START, Event.EMERGENCY]


def test_gpio_buttons_cleans_up_on_close():
    gpio = FakeGPIO()
    buttons = GpioButtons(11, 13, gpio=gpio)
    buttons.close()
    assert gpio.cleanup_calls == [[11, 13]]


def test_gpio_buttons_requires_jetson_gpio_when_not_injected(monkeypatch):
    real_import = __import__("builtins").__import__

    def import_without_jetson(name, *args, **kwargs):
        if name == "Jetson.GPIO":
            raise ImportError("not installed")
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr("builtins.__import__", import_without_jetson)
    with pytest.raises(RuntimeError, match="Jetson.GPIO"):
        GpioButtons(11, 13)
