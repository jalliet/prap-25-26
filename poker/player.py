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
    
    This class maintains both the long-term identity of a participant and their
    volatile state within a single hand (cards, bets, status).
    
    Attributes:
        player_id: Unique identifier for the player.
        name: Display name.
        seat: Seat position at table (0-indexed).
        stack: ChipStack representing player's total chips.
        hole_cards: List of 2 Cards, or None if unknown/not dealt.
        current_bet: Chips bet in the current betting round.
        status: Current PlayerStatus (Active, Folded, etc.).
        position_label: Table position label (BTN, SB, BB, UTG, etc.).
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
        """Resets volatile per-hand state for a fresh deal."""
        self.hole_cards = None
        self.current_bet = 0
        self.status = PlayerStatus.ACTIVE
        self.position_label = ""
        self.actions = []
    
    def set_hole_cards(self, cards: List[Card]) -> None:
        """Assigns the player's hole cards."""
        if len(cards) != 2:
            raise ValueError(f"Must have exactly 2 hole cards, got {len(cards)}")
        self.hole_cards = cards
    
    def fold(self) -> None:
        """Marks the player as folded and out of the hand."""
        self.status = PlayerStatus.FOLDED
        self.actions.append("fold")
    
    def bet(self, amount: int) -> int:
        """
        Commits chips from the player's stack to the pot.
        
        Args:
            amount: The number of chips the player intends to bet.
        Returns:
            The actual amount bet (capped by stack size if all-in).
        """

        max_bet = self.stack.total
        actual_bet = min(amount, max_bet)

        # Update stack: Deduct amount by recreating stack from new total
        # Ideally, we would remove specific chips, but total-based update works for virtual stacks
        new_total = self.stack.total - actual_bet 
        self.stack = ChipStack.from_total(new_total)
        
        self.current_bet += actual_bet
        
        # Check if all-in
        if self.stack.is_empty():
            self.status = PlayerStatus.ALL_IN
        
        return actual_bet
    
    def collect_winnings(self, amount: int) -> None:
        """Adds winnings to the player's stack."""
        new_total = self.stack.total + amount
        self.stack = ChipStack.from_total(new_total)
    
    def is_active(self) -> bool:
        """Returns True if the player can still take actions (not folded, not all-in)."""
        return self.status == PlayerStatus.ACTIVE
    
    def is_in_hand(self) -> bool:
        """Returns True if the player is still competing for the pot (Active or All-in)."""
        return self.status in (PlayerStatus.ACTIVE, PlayerStatus.ALL_IN)
    
    def __str__(self) -> str:
        """
        Returns a compact string representation matching sample format.
        Example: P0 Hero (Button, Seat 2): Stack 985, Bet 35, Cards A♦ Q♥, Active
        """
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
        """
        Returns an unambiguous string representation for debugging.
        Format: Player(id=X, name='Y', seat=Z, stack=..., status=...)
        """
        return f"Player(id={self.player_id}, name='{self.name}', seat={self.seat}, stack={self.stack}, status={self.status.name})"