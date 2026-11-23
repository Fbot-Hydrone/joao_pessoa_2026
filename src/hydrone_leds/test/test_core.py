import pytest

from hydrone_leds.core import COLORS, FakeGpio, RgbLeds, SysfsGpio, color_name, parse_color
from hydrone_leds.cli import main as cli


def test_parse_names_letters_and_mixes():
    assert parse_color('white') == (1, 1, 1)
    assert parse_color('red blue') == (1, 0, 1) == parse_color('rb') == parse_color('r,b')
    assert parse_color(['green', 'b']) == (0, 1, 1)
    assert parse_color('OFF') == (0, 0, 0)


@pytest.mark.parametrize('bad', ['', 'pink', 'rgx', 'off red', 'white blue'])
def test_parse_rejects(bad):
    with pytest.raises(ValueError):
        parse_color(bad)


def test_all_eight_combinations_have_a_name():
    states = {(r, g, b) for r in (0, 1) for g in (0, 1) for b in (0, 1)}
    assert set(COLORS.values()) == states
    for name, s in COLORS.items():
        assert color_name(s) == name


def test_fake_backend():
    leds = RgbLeds(FakeGpio())
    leds.set('cyan')
    assert leds.get() == (0, 1, 1)
    leds.off()
    assert leds.get() == (0, 0, 0)


def _fake_sysfs(tmp_path, gpios):
    # a kernel would create gpioN/ on export; pre-create them instead
    (tmp_path / 'export').write_text('')
    for g in gpios:
        d = tmp_path / ('gpio%d' % g)
        d.mkdir()
        (d / 'direction').write_text('in')
        (d / 'value').write_text('0')


def test_sysfs_writes_direction_with_level(tmp_path):
    gpios = (149, 200, 38)
    _fake_sysfs(tmp_path, gpios)
    gpio = SysfsGpio(gpios, root=str(tmp_path))
    assert gpio.read() == (0, 0, 0)  # inputs read as off
    gpio.write((1, 0, 1))
    assert [(tmp_path / ('gpio%d' % g) / 'direction').read_text() for g in gpios] == \
        ['high', 'low', 'high']


def test_sysfs_reads_outputs(tmp_path):
    _fake_sysfs(tmp_path, (1, 2, 3))
    for g, v in zip((1, 2, 3), '101'):
        (tmp_path / ('gpio%d' % g) / 'direction').write_text('out')
        (tmp_path / ('gpio%d' % g) / 'value').write_text(v)
    assert SysfsGpio((1, 2, 3), root=str(tmp_path)).read() == (1, 0, 1)


def test_sysfs_missing_pin_times_out(tmp_path):
    # export "works" but the pin never shows up: fail, don't hang
    (tmp_path / 'export').write_text('')
    gpio = SysfsGpio((7, 8, 9), root=str(tmp_path), udev_wait=0.05)
    with pytest.raises(FileNotFoundError):
        gpio.write((1, 1, 1))
    assert (tmp_path / 'export').read_text() == '7'


def test_cli(capsys):
    assert cli(['--backend', 'fake', 'red', 'green']) == 0
    assert capsys.readouterr().out.startswith('yellow')
    assert cli(['--backend', 'fake', 'pink']) == 1
    with pytest.raises(SystemExit):
        cli(['--backend', 'fake', '--gpios', '1,2', 'red'])
