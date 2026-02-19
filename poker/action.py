from enum import Enum
from dataclasses import dataclass


class ActionType(Enum):
    """Types of actions a player can take."""
    FOLD = "fold"
    CHECK = "check"
    CALL = "call"
    BET = "bet"
    RAISE = "raise"
    ALL_IN = "all_in"
    
    # Forced bets
    POST_SB = "post_sb"
    POST_BB = "post_bb"
    POST_ANTE = "post_ante"


@dataclass
class Action:
    """
    Represents a player's action within a poker hand.
    
    This artifact serves as the primary data interface for player moves, capturing
    who acted, what they did, and the financial implication (amount).
    
    Attributes:
        player_id: Unique identifier of the player who took the action.
        action_type: The specific type of move (e.g., FOLD, RAISE).
        amount: The number of chips committed in this specific action (0 for fold/check).
                Note: This is the incremental amount added to the pot, not necessarily the total bet.
    """
    player_id: int
    action_type: ActionType
    amount: int = 0
    
    def __str__(self) -> str:
        """
        Returns a compact, human-readable string representation (e.g., "P0 raise 30").
        Useful for logging and UI display.
        """
        if self.amount > 0:
            return f"P{self.player_id} {self.action_type.value} {self.amount}"
        return f"P{self.player_id} {self.action_type.value}"
    
    def __repr__(self) -> str:
        """
        Returns an unambiguous string representation for debugging and agent context.
        Format: Action(player_id=X, action_type=Y, amount=Z)
        """
        return f"Action(player_id={self.player_id}, action_type={self.action_type.name}, amount={self.amount})"
