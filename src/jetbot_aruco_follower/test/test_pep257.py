"""Run the ROS 2 docstring style checker."""

from ament_pep257.main import main
import pytest


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    """Check public docstrings with the ament profile."""
    assert main(argv=['.', 'test']) == 0, (
        'Found code style errors / warnings')
