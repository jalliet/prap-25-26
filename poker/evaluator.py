"""Hand evaluator wrapper around phevaluator.

Exposes a single function best_hand_score(hole, community) that returns
phevaluator's int score (lower = stronger). Split-pot detection at the
SHOWDOWN level uses score equality.
"""
from typing import List
from phevaluator import evaluate_cards
from poker.card import Card


def _card_to_str(card: Card) -> str:
    """Convert our Card to phevaluator's string code (e.g. 'Ah', 'Td', '2c')."""
    return f"{card.rank.code}{card.suit.code.lower()}"


def best_hand_score(hole_cards: List[Card], community_cards: List[Card]) -> int:
    """Return phevaluator's best 5-of-7 score for this player.

    Lower is stronger. Two players with equal scores split the pot.
    """
    if hole_cards is None or len(hole_cards) != 2:
        raise ValueError("Need exactly 2 hole cards")
    if len(community_cards) != 5:
        raise ValueError(f"Need exactly 5 community cards, got {len(community_cards)}")
    cards = [_card_to_str(c) for c in hole_cards] + [_card_to_str(c) for c in community_cards]
    return evaluate_cards(*cards)
