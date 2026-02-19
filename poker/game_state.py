from enum import Enum, auto
from typing import List, Callable, Optional
from poker.player import Player
from poker.card import Card, Deck
from poker.chips import ChipStack
from poker.action import Action, ActionType

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
    
    This class maintains the ground truth of the game (players, pot, cards, phase)
    and emits signals when this state changes, allowing UI and Vision systems to react.
    """
    def __init__(self):
        self.players: List[Player] = []
        self.pot: ChipStack = ChipStack()
        self.community_cards: List[Card] = []
        self.deck: Deck = Deck()
        self.phase: GamePhase = GamePhase.PRE_FLOP
        self.current_player_index: int = 0
        self.dealer_index: int = 0
        
        # Betting state
        self.current_bet_amount: int = 0
        self.last_raiser_index: int = -1  # Index of player who made the last aggressive action
        self.players_acted_in_round: int = 0 # Count of players who acted this round

        # Vision flags
        self.hand_detection_active: bool = False
        self.card_detection_active: bool = False

        # Signals
        self.on_phase_change = Signal()
        self.on_pot_change = Signal()
        self.on_card_detection_required = Signal()
        self.on_player_action = Signal()  # Emitted with an Action object
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
        self.current_bet_amount = 0
        self.last_raiser_index = -1
        self.players_acted_in_round = 0
        
        # Rotate dealer button
        self.dealer_index = (self.dealer_index + 1) % len(self.players)
        
        # Reset players status
        for player in self.players:
            player.reset_for_new_hand()
            
        # Set initial current player (Player after dealer = SB)
        # Note: In heads-up (2 players), Dealer is SB. Simple logic, assuming 2+ players for standard rotation.
        # Robust implementation would handle heads-up rules.
        self.current_player_index = self.dealer_index
        self.next_turn() # Advance to SB
        self.next_turn() # Advance to BB (to start action UTG, we need more calls)
        # Simplicity: just start with player after dealer for now
        self.current_player_index = (self.dealer_index + 1) % len(self.players)
        
        # In a real game, SB and BB would post blinds here.
        # For now, we'll assume they are posted or handled by process_action later.

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

    def process_action(self, action: Action):
        """
        Handles a player action and updates the game state.
        
        This method takes a fully formed Action artifact, applies the financial
        implications, and advances the game flow.
        
        Args:
            action: The Action object containing player_id, type, and amount.
        """
        # Resolve player from ID
        player = next((p for p in self.players if p.player_id == action.player_id), None)
        if not player:
            print(f"Error: Player ID {action.player_id} not found.")
            return

        if player != self.current_player:
            print(f"Error: It is not {player.name}'s turn.")
            return

        action_type = action.action_type
        amount = action.amount
        
        # Validate and Execute Action
        if action_type == ActionType.FOLD:
            player.fold()
            
        elif action_type == ActionType.CHECK:
            if player.current_bet < self.current_bet_amount:
                print("Error: Cannot check, must call.")
                return 
            pass # Check is valid
            
        elif action_type == ActionType.CALL:
            to_call = self.current_bet_amount - player.current_bet
            actual_bet = player.bet(to_call)
            self.update_pot(actual_bet)
            # Update action amount to reflect actual chips committed (for logging/UI)
            action.amount = actual_bet 
            
        elif action_type == ActionType.RAISE or action_type == ActionType.BET:
             if amount <= 0:
                  print(f"Error: Raise amount {amount} must be positive.")
                  return
             
             # Target total bet = Current highest bet + Raise amount
             new_total_bet = self.current_bet_amount + amount
             
             # Amount to add from stack = Target - What player already has in front
             to_add = new_total_bet - player.current_bet
             
             actual_bet = player.bet(to_add)
             self.update_pot(actual_bet)
             
             # Update table state
             if player.current_bet > self.current_bet_amount:
                 self.current_bet_amount = player.current_bet
                 self.last_raiser_index = self.current_player_index
                 self.players_acted_in_round = 0 
            
        self.players_acted_in_round += 1
        
        # Emit the Action artifact directly
        self.on_player_action.emit(action)
        
        if self._is_round_complete():
            self._advance_phase()
        else:
            self.next_turn()

    def _is_round_complete(self) -> bool:
        """Checks if the betting round is complete."""
        active_players = self.get_eligible_players()
        if len(active_players) < 2:
            return True # Everyone else folded
            
        # Round is complete if:
        # 1. Everyone active has acted at least once (or we are back to raiser)
        # 2. Everyone active has matched the current bet
        
        # If no one raised, we need everyone to check
        if self.last_raiser_index == -1:
             # Pre-flop is special (BB is "raiser"), but simplifying for now
             if self.players_acted_in_round >= len(active_players):
                 return True
        else:
            # If someone raised, we continue until everyone calls or folds
            # Checking if current player matches the bet is handled by the loop logic usually
            # But here we just check if everyone matches
            all_matched = all(p.current_bet == self.current_bet_amount for p in active_players)
            if all_matched and self.players_acted_in_round >= len(active_players):
                return True
                
        return False

    def _advance_phase(self):
        """
        Moves the game to the next phase (Pre-Flop -> Flop -> Turn -> River -> Showdown).
        
        This transition resets betting parameters and triggers card dealing based on
        a declarative configuration of phase requirements.
        """
        # Configuration: Cards to deal for each phase
        PHASE_DEAL_COUNTS = {
            GamePhase.FLOP: 3,
            GamePhase.TURN: 1,
            GamePhase.RIVER: 1
        }

        # Determine next phase using the enum order
        phases = list(GamePhase)
        try:
            current_index = phases.index(self.phase)
            next_index = current_index + 1
            
            if next_index < len(phases):
                self.phase = phases[next_index]
            else:
                # End of hand loop (Showdown -> PreFlop or just Stop)
                print("Hand complete. Waiting for new hand.")
                return 
        except ValueError:
            print(f"Error: Current phase {self.phase} not found in enum list.")
            return

        # Phase-specific logic (Dealing cards)
        cards_to_deal = PHASE_DEAL_COUNTS.get(self.phase, 0)
        if cards_to_deal > 0:
            self.deal_community_cards(cards_to_deal)
        
        if self.phase == GamePhase.SHOWDOWN:
            # Handle showdown logic here (determine winner)
            pass
            
        # Reset betting for new phase
        self.current_bet_amount = 0
        self.last_raiser_index = -1
        self.players_acted_in_round = 0
        for p in self.players:
            p.current_bet = 0
            
        # Reset turn to first active player after dealer
        self.current_player_index = self.dealer_index
        self.next_turn()
        
        self.on_phase_change.emit(self.phase)
