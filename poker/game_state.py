from enum import Enum, auto
from typing import List, Callable, Optional
from poker.player import Player
from poker.card import Card, Deck
from poker.chips import ChipStack

class GamePhase(Enum):
    PRE_FLOP = auto()
    FLOP = auto()
    TURN = auto()
    RIVER = auto()
    SHOWDOWN = auto()

class Signal:
    """Simple signal implementation for event handling."""
    def __init__(self):
        self._callbacks: List[Callable] = []

    def connect(self, callback: Callable):
        self._callbacks.append(callback)

    def emit(self, *args, **kwargs):
        for callback in self._callbacks:
            callback(*args, **kwargs)

class GameState:
    """
    Central class to orchestrate the poker game flow and trigger vision events.
    """
    def __init__(self):
        self.players: List[Player] = []
        self.pot: ChipStack = ChipStack()
        self.community_cards: List[Card] = []
        self.deck: Deck = Deck()
        self.phase: GamePhase = GamePhase.PRE_FLOP
        self.current_player_index: int = 0
        
        # Vision flags
        self.hand_detection_active: bool = False
        self.card_detection_active: bool = False

        # Signals
        self.on_phase_change = Signal()
        self.on_pot_change = Signal()
        self.on_card_detection_required = Signal()

    def start_new_hand(self):
        """Resets state for a new hand."""
        self.deck.reset()
        self.deck.shuffle()
        self.community_cards = []
        self.pot = ChipStack()
        self.phase = GamePhase.PRE_FLOP
        self.current_player_index = 0
        
        # Reset players status
        for player in self.players:
            player.reset_for_new_hand()

        self.on_phase_change.emit(self.phase)

    def deal_community_cards(self, count: int):
        """Deals cards to the board and triggers detection."""
        new_cards = self.deck.deal(count)
        self.community_cards.extend(new_cards)
        self.on_card_detection_required.emit(new_cards)

    def update_pot(self, amount: int):
        """
        Updates pot size.
        Args:
            amount: Amount to add to pot.
        """
        # Create a stack from amount (assuming optimal chip breakdown for now)
        # In reality, this might come from specific chip detection
        chips_added = ChipStack.from_total(amount)
        self.pot.add_stack(chips_added)
        self.on_pot_change.emit(self.pot.total)

    def process_action(self, player: Player, action_type: str, amount: int = 0):
        """
        Handles player actions (Check, Call, Raise, Fold).
        Args:
            player: The player performing the action
            action_type: 'check', 'call', 'raise', 'fold'
            amount: The amount involved (for call/raise)
        """
        # Placeholder for action logic
        pass
