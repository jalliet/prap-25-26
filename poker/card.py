from enum import Enum
from typing import Optional
import random


class Suit(Enum):
    """
    Represents the four suits in a standard deck of cards.
    """
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
        """
        Factory method to create a Suit from a short code (H, D, C, S).
        """
        for suit in cls:
            if suit.code == code.upper():
                return suit
        raise ValueError(f"Invalid suit code: {code}")


class Rank(Enum):
    """
    Represents card ranks with numeric values for comparison.
    Ace is considered high (14).
    """
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
        """Returns the single-character code for this rank (e.g., 'T', 'A')."""
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
        """
        Factory method to create a Rank from a code (2-9, T, J, Q, K, A).
        """
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
    Represents a standard playing card with a specific Rank and Suit.
    
    This class is immutable and hashable, making it suitable for use in sets
    and dictionary keys (e.g., for hand analysis or tracking dealt cards).
    """
    
    def __init__(self, rank: Rank, suit: Suit):
        self.rank = rank
        self.suit = suit
    
    def __str__(self) -> str:
        """Returns a human-readable string representation (e.g., 'A♥' or 'K♠')."""
        return f"{self.rank}{self.suit}"
    
    def __repr__(self) -> str:
        """
        Returns an unambiguous string representation for debugging.
        Format: Card(rank_code, suit_code) -> e.g., Card(AH)
        """
        return f"Card({self.rank.code}{self.suit.code})"
    
    def __eq__(self, other) -> bool:
        if not isinstance(other, Card):
            return False
        return self.rank == other.rank and self.suit == other.suit
    
    def __hash__(self) -> int:
        """
        Enables Cards to be used in sets and as dict keys.
        Crucial for duplicate detection and fast lookups.
        """
        return hash((self.rank, self.suit))
    
    @classmethod
    def from_string(cls, card_str: str) -> 'Card':
        """
        Factory method to create a Card from standard notation (rank code + suit code).
        
        Args:
            card_str: A string like 'AH', 'KS', '2D', 'TC'.
                      Rank codes: 2-9, T, J, Q, K, A
                      Suit codes: H (Hearts), D (Diamonds), C (Clubs), S (Spades)
        Returns:
            A new Card instance.
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
    Represents a standard 52-card deck with shuffle and deal functionality.
    
    State:
        cards: List of Card objects currently remaining in the deck.
        dealt_cards: List of Card objects that have been dealt since the last reset.
    """
    
    def __init__(self):
        self.cards: list[Card] = []
        self.dealt_cards: list[Card] = []
        self.reset()
    
    def reset(self):
        """Resets the deck to a fresh, ordered state with all 52 cards."""
        self.cards = [
            Card(rank, suit)
            for suit in Suit
            for rank in Rank
        ]
        self.dealt_cards = []
    
    def shuffle(self):
        """Randomises the order of the remaining cards in the deck."""
        random.shuffle(self.cards)
    
    def deal(self, n: int = 1) -> list[Card]:
        """
        Deals a specified number of cards from the top of the deck.
        
        Args:
            n: Number of cards to deal.
        Returns:
            List of dealt Card objects.
        Raises:
            ValueError: If fewer than n cards remain in the deck.
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
        """Deals a single card from the deck."""
        return self.deal(1)[0]
    
    def remaining(self) -> int:
        """Returns the number of cards currently remaining in the deck."""
        return len(self.cards)
    
    def __len__(self) -> int:
        return len(self.cards)
    
    def __str__(self) -> str:
        return f"Deck({self.remaining()} cards remaining)"


def make_card(card_str: str) -> Card:
    """
    Convenience factory function to create a Card from code notation.
    """
    return Card.from_string(card_str)
