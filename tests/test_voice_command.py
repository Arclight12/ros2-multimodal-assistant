from __future__ import annotations

import pytest
from assistant_interaction.voice_command import match_class


def test_match_sharpener():
    assert match_class("pick up the sharpener") == "sharpener"
    assert match_class("give me the pencil sharpener please") == "sharpener"


def test_match_eraser():
    assert match_class("pick the eraser") == "eraser"
    assert match_class("pass me the rubber") == "eraser"


def test_match_thermocol():
    assert match_class("pick up the thermocol block") == "thermocol"
    assert match_class("grab the foam") == "thermocol"


def test_match_blocks():
    assert match_class("please pick block a") == "block_a"
    assert match_class("grab cube b") == "block_b"
    assert match_class("give me letter c") == "block_c"
    assert match_class("pick c") == "block_c"


def test_no_false_positive_on_article_a():
    # 'a' in 'pick a sharpener' should not match block_a
    assert match_class("pick a sharpener") == "sharpener"


def test_ambiguous_two_classes_returns_none():
    assert match_class("pick up block a and sharpener") is None
