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
        self.dealer_index: int = 0
        
        # Vision flags
        self.hand_detection_active: bool = False
        self.card_detection_active: bool = False

        # Signals
        self.on_phase_change = Signal()
        self.on_pot_change = Signal()
        self.on_card_detection_required = Signal()
        self.on_player_action = Signal()  # Emitted when a player acts
        self.on_turn_change = Signal()    # Emitted when turn passes to next player

    def add_player(self, player: Player):
        """Adds a player to the game."""
        self.players.append(player)
        # Sort by seat to ensure consistent order
        self.players.sort(key=lambda p: p.seat)

    def remove_player(self, player_id: int):
        """Removes a player by ID."""
        self.players = [p for p in self.players if p.player_id != player_id]

    def get_eligible_players(self) -> List[Player]:
        """Returns list of players who can still act (not folded, not all-in)."""
        return [p for p in self.players if p.is_active()]

    def get_players_in_hand(self) -> List[Player]:
        """Returns list of players still in the hand (active + all-in)."""
        return [p for p in self.players if p.is_in_hand()]

    @property
    def current_player(self) -> Optional[Player]:
        """Returns the player whose turn it is."""
        if not self.players:
            return None
        return self.players[self.current_player_index]

    def start_new_hand(self):
        """Resets state for a new hand."""
        if len(self.players) < 2:
            print("Not enough players to start a hand.")
            return

        self.deck.reset()
        self.deck.shuffle()
        self.community_cards = []
        self.pot = ChipStack()
        self.phase = GamePhase.PRE_FLOP
        
        # Rotate dealer button
        self.dealer_index = (self.dealer_index + 1) % len(self.players)
        
        # Reset players status
        for player in self.players:
            player.reset_for_new_hand()
            
        # Set initial current player (Player after dealer = SB)
        # Note: In heads-up (2 players), Dealer is SB. Simple logic, assuming 2+ players for standard rotation, but just like basic rotation. 
        # Robust implementation would handle heads-up rules.
        self.current_player_index = self.dealer_index
        self.next_turn() # Advance to SB
        self.next_turn() # Advance to BB (to start action UTG, we need more calls)
        # Simplicity: just start with player after dealer for now
        self.current_player_index = (self.dealer_index + 1) % len(self.players)

        self.on_phase_change.emit(self.phase)
        self.on_turn_change.emit(self.current_player)

    def next_turn(self):
        """Advances turn to the next active player."""
        if not self.get_eligible_players():
            # No one can act (all folded or all-in)
            return

        start_index = self.current_player_index
        
        while True:
            self.current_player_index = (self.current_player_index + 1) % len(self.players)
            player = self.players[self.current_player_index]
            
            if player.is_active():
                break
                
            # Safety break if we looped all the way around
            if self.current_player_index == start_index:
                break
        
        self.on_turn_change.emit(self.current_player)

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
