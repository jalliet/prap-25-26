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
    Represent player actions in poker hand.
    Attributes:
        player_id: ID of player who took action
        action_type: Type of action taken
        amount: Chips committed (0 for fold/check)
    """
    player_id: int
    action_type: ActionType
    amount: int = 0
    
    def __str__(self) -> str:
        """Compact representation: P0 raise 30"""
        if self.amount > 0:
            return f"P{self.player_id} {self.action_type.value} {self.amount}"
        return f"P{self.player_id} {self.action_type.value}"
    
    def __repr__(self) -> str:
        return f"Action({self.player_id}, {self.action_type.name}, {self.amount})"
