"""Spoken object-name grammar and matching for the voice command path.

The Vosk grammar restricts recognition to a small vocabulary, and
``match_class`` maps recognised text onto a detector class. Pure helpers
with no ROS or vosk imports, so they stay testable without hardware.
"""

from __future__ import annotations

import json
import re

DEFAULT_ALIASES: dict[str, tuple[str, ...]] = {
    "cup": ("cup", "mug"),
    "water_bottle": ("water bottle", "bottle"),
    "book": ("book",),
    "pen": ("pen",),
    "phone": ("phone", "mobile", "cell phone"),
    "ball": ("ball",),
}

COMMAND_WORDS = (
    "pick", "up", "grab", "get", "me", "the", "a", "please", "give",
)


def grammar_json(
    aliases: dict[str, tuple[str, ...]] = DEFAULT_ALIASES,
) -> str:
    """Return the Vosk grammar: every alias, its words, and command words."""
    words: list[str] = []
    for phrases in aliases.values():
        for phrase in phrases:
            words.append(phrase)
            words.extend(phrase.split())
    words.extend(COMMAND_WORDS)
    words.append("[unk]")
    # dict.fromkeys drops duplicates while keeping first-seen order.
    return json.dumps(list(dict.fromkeys(words)))


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
