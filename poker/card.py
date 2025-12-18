from enum import Enum
from typing import Optional
import random


class Suit(Enum):
    HEARTS = ("H", "♥")
    DIAMONDS = ("D", "♦")
    CLUBS = ("C", "♣")
    SPADES = ("S", "♠")
    
    def __init__(self, code: str, symbol: str):
        self.code = code
        self.symbol = symbol
    
    def __str__(self) -> str:
        return self.symbol
    
    @classmethod
    def from_code(cls, code: str) -> 'Suit':
        """Create Suit from code (H, D, C, S)."""
        for suit in cls:
            if suit.code == code.upper():
                return suit
        raise ValueError(f"Invalid suit code: {code}")


class Rank(Enum):
    """Card ranks with numeric values for comparison."""
    TWO = 2
    THREE = 3
    FOUR = 4
    FIVE = 5
    SIX = 6
    SEVEN = 7
    EIGHT = 8
    NINE = 9
    TEN = 10
    JACK = 11
    QUEEN = 12
    KING = 13
    ACE = 14
    
    @property
    def code(self) -> str:
        """Get the single-character code for this rank."""
        codes = {
            2: "2", 3: "3", 4: "4", 5: "5", 6: "6",
            7: "7", 8: "8", 9: "9", 10: "T",
            11: "J", 12: "Q", 13: "K", 14: "A"
        }
        return codes[self.value]
    
    def __str__(self) -> str:
        return self.code
    
    def __lt__(self, other: 'Rank') -> bool:
        return self.value < other.value
    
    def __le__(self, other: 'Rank') -> bool:
        return self.value <= other.value
    
    def __gt__(self, other: 'Rank') -> bool:
        return self.value > other.value
    
    def __ge__(self, other: 'Rank') -> bool:
        return self.value >= other.value
    
    @classmethod
    def from_code(cls, code: str) -> 'Rank':
        """Create Rank from code (2-9, T, J, Q, K, A)."""
        code_map = {
            "2": cls.TWO, "3": cls.THREE, "4": cls.FOUR,
            "5": cls.FIVE, "6": cls.SIX, "7": cls.SEVEN,
            "8": cls.EIGHT, "9": cls.NINE, "T": cls.TEN,
            "J": cls.JACK, "Q": cls.QUEEN, "K": cls.KING, "A": cls.ACE
        }
        rank = code_map.get(code.upper())
        if rank is None:
            raise ValueError(f"Invalid rank code: {code}")
        return rank


class Card:
    """
    Represents a playing card with rank and suit.
    """
    
    def __init__(self, rank: Rank, suit: Suit):
        self.rank = rank
        self.suit = suit
    
    def __str__(self) -> str:
        """String representation: e.g., 'A♥' or 'K♠'"""
        return f"{self.rank}{self.suit}"
    
    def __repr__(self) -> str:
        return f"Card({self.rank.code}{self.suit.code})"
    
    def __eq__(self, other) -> bool:
        if not isinstance(other, Card):
            return False
        return self.rank == other.rank and self.suit == other.suit
    
    def __hash__(self) -> int:
        """
        Enable Cards to be used in sets and as dict keys.
        For dupl detection, tracking dealt cards, fast lookups.
        """
        return hash((self.rank, self.suit))
    
    @classmethod
    def from_string(cls, card_str: str) -> 'Card':
        """
        Create Card from code notation (rank + suit code).
        Examples: 'AH', 'KS', '2D', 'TC'
        Rank codes: 2-9, T, J, Q, K, A
        Suit codes: H (Hearts), D (Diamonds), C (Clubs), S (Spades)
        """
        card_str = card_str.strip().upper()
        
        if len(card_str) != 2:
            raise ValueError(f"Invalid card string: {card_str} (expected 2 characters)")
        
        rank_code = card_str[0]
        suit_code = card_str[1]
        
        rank = Rank.from_code(rank_code)
        suit = Suit.from_code(suit_code)
        
        return cls(rank, suit)


class Deck:
    """
    Standard 52-card deck with shuffle and deal functionality.
    """
    
    def __init__(self):
        self.cards: list[Card] = []
        self.dealt_cards: list[Card] = []
        self.reset()
    
    def reset(self):
        """Create a fresh 52-card deck."""
        self.cards = [
            Card(rank, suit)
            for suit in Suit
            for rank in Rank
        ]
        self.dealt_cards = []
    
    def shuffle(self):
        """Shuffle the remaining cards in the deck."""
        random.shuffle(self.cards)
    
    def deal(self, n: int = 1) -> list[Card]:
        """
        Deal a specified number of cards from the deck.
        Returns:
            List of dealt cards
        Raises:
            ValueError: If not enough cards remain
        """
        if n > len(self.cards):
            raise ValueError(
                f"Cannot deal {n} cards, only {len(self.cards)} remaining"
            )
        
        dealt = self.cards[:n]
        self.cards = self.cards[n:]
        self.dealt_cards.extend(dealt)
        
        return dealt
    
    def deal_one(self) -> Card:
        """Deal a single card from the deck."""
        return self.deal(1)[0]
    
    def remaining(self) -> int:
        """Return number of cards remaining in deck."""
        return len(self.cards)
    
    def __len__(self) -> int:
        return len(self.cards)
    
    def __str__(self) -> str:
        return f"Deck({self.remaining()} cards remaining)"


def make_card(card_str: str) -> Card:
    """
    Create a Card from code notation.
    """
    return Card.from_string(card_str)
