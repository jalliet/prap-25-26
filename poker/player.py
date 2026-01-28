from enum import Enum
from typing import Optional, List
from poker.card import Card
from poker.chips import ChipStack


class PlayerStatus(Enum):
    """Player status in a hand."""
    ACTIVE = "active"       # Still in the hand, can act
    FOLDED = "folded"       # Out of the hand
    ALL_IN = "all_in"       # All chips committed, cannot act further
    SITTING_OUT = "sitting_out"  # Not participating in current hand


class Player:
    """
    Represents a poker player at the table.
    
    Attributes:
        player_id: Unique identifier for the player
        name: Display name
        seat: Seat position at table (0-indexed)
        stack: ChipStack representing player's chips
        hole_cards: List of 2 Cards, or None if unknown/not dealt
        current_bet: Chips bet in current betting round
        status: PlayerStatus enum value
        position_label: Table position (BTN, SB, BB, UTG, etc.)
    """
    
    def __init__(
        self,
        player_id: int,
        name: str,
        seat: int,
        stack: ChipStack = None,
        is_robot: bool = False
    ):
        self.player_id = player_id
        self.name = name
        self.seat = seat
        self.stack = stack if stack else ChipStack()
        self.is_robot = is_robot 
        
        # Per-hand state (reset each hand)
        self.hole_cards: Optional[List[Card]] = None
        self.current_bet: int = 0
        self.status: PlayerStatus = PlayerStatus.SITTING_OUT
        self.position_label: str = ""
        self.actions: List[str] = []  # Action history for current hand
    
    def reset_for_new_hand(self) -> None:
        """Reset per-hand state for a new hand."""
        self.hole_cards = None
        self.current_bet = 0
        self.status = PlayerStatus.ACTIVE
        self.position_label = ""
        self.actions = []
    
    def set_hole_cards(self, cards: List[Card]) -> None:
        """Set the player's hole cards."""
        if len(cards) != 2:
            raise ValueError(f"Must have exactly 2 hole cards, got {len(cards)}")
        self.hole_cards = cards
    
    def fold(self) -> None:
        """Mark player as folded."""
        self.status = PlayerStatus.FOLDED
        self.actions.append("fold")
    
    def bet(self, amount: int) -> int:
        """
        Args:
            amount: Amount to bet
        Returns:
            Actual amount bet (may be less if all-in)
        """

        max_bet = self.stack.total
        actual_bet = min(amount, max_bet)

        new_total = self.stack.total - actual_bet # Deduct from, then redefine stack
        self.stack = ChipStack.from_total(new_total)
        
        self.current_bet += actual_bet
        
        # Check if all-in
        if self.stack.is_empty():
            self.status = PlayerStatus.ALL_IN
        
        return actual_bet
    
    def collect_winnings(self, amount: int) -> None:
        """Add winnings to player's stack."""
        new_total = self.stack.total + amount
        self.stack = ChipStack.from_total(new_total)
    
    def is_active(self) -> bool:
        """Check if player can still act in the hand."""
        return self.status == PlayerStatus.ACTIVE
    
    def is_in_hand(self) -> bool:
        """Check if player is still competing for the pot."""
        return self.status in (PlayerStatus.ACTIVE, PlayerStatus.ALL_IN)
    
    def __str__(self) -> str:
        """Compact string representation matching sample format."""
        # P0 Hero (Button, Seat 2): Stack 985, Bet 35, Cards A♦ Q♥, Active
        cards_str = "unknown"
        if self.hole_cards:
            cards_str = " ".join(str(c) for c in self.hole_cards)
        elif self.is_robot and self.hole_cards is None:
            cards_str = "not dealt"
        
        position_str = f"{self.position_label}, " if self.position_label else ""
        robot_tag = " (Robot)" if self.is_robot else ""
        
        return (
            f"P{self.player_id} {self.name}{robot_tag} "
            f"({position_str}Seat {self.seat}): "
            f"Stack {self.stack.total}, Bet {self.current_bet}, "
            f"Cards {cards_str}, {self.status.value.capitalize()}"
        )
    
    def __repr__(self) -> str:
        return f"Player({self.player_id}, '{self.name}', seat={self.seat})"