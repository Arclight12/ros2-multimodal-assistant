"""Map transcribed speech onto a detector class name (pure, no ROS)."""

from __future__ import annotations

import re

DEFAULT_ALIASES: dict[str, tuple[str, ...]] = {
    "sharpener": ("sharpener", "pencil sharpener", "cutter", "softener", "shop nor", "shopner", "sharpner"),

    "eraser": ("eraser", "rubber"),
    "thermocol": ("thermocol", "foam", "white block", "thermocol block"),
    "block_a": ("block a", "cube a", "box a", "letter a", "item a", "object a"),
    "block_b": ("block b", "cube b", "box b", "letter b", "item b", "object b", "b"),
    "block_c": ("block c", "cube c", "box c", "letter c", "item c", "object c", "c"),
    "cup": ("cup", "mug"),
    "water_bottle": ("water bottle", "bottle"),
    "book": ("book",),
    "pen": ("pen",),
    "phone": ("phone", "mobile", "cell phone"),
    "ball": ("ball",),
    "toy": ("toy",),
}



def match_class(
    text: str, aliases: dict[str, tuple[str, ...]] = DEFAULT_ALIASES
) -> str | None:
    """Return the single detector class named in ``text``, else ``None``.

    Matches whole words or phrases only. A longer alias claims its span, so
    "water bottle" does not also count as a shorter alias. Two different
    classes in one utterance are ambiguous and return ``None``.
    """
    lowered = text.lower()
    hits: list[tuple[int, int, str]] = []
    for cls, phrases in aliases.items():
        for phrase in phrases:
            pattern = r"\b" + re.escape(phrase) + r"\b"
            for match in re.finditer(pattern, lowered):
                hits.append((match.start(), match.end(), cls))
    # Longest first, so shorter aliases inside a claimed span are dropped.
    hits.sort(key=lambda hit: hit[1] - hit[0], reverse=True)
    claimed: list[tuple[int, int]] = []
    classes: set[str] = set()
    for start, end, cls in hits:
        if any(start < c_end and c_start < end for c_start, c_end in claimed):
            continue
        claimed.append((start, end))
        classes.add(cls)
    return classes.pop() if len(classes) == 1 else None
