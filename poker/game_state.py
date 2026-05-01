from dataclasses import dataclass
from enum import Enum, auto
from typing import List, Callable, Optional
from poker.player import Player, PlayerStatus
from poker.card import Card, Deck
from poker.chips import ChipStack
from poker.action import Action, ActionType


@dataclass
class BlindStructure:
    """Blind levels for cash-game play. Tournament-style escalation out of scope."""
    small_blind: int = 1
    big_blind: int = 2


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
    def __init__(self, blinds: BlindStructure = None):
        self.players: List[Player] = []
        self.pot: ChipStack = ChipStack()
        self.community_cards: List[Card] = []
        self.deck: Deck = Deck()
        self.phase: GamePhase = GamePhase.PRE_FLOP
        self.current_player_index: int = 0
        self.dealer_index: int = 0

        # Blind structure (cash game; no escalation)
        self.blinds: BlindStructure = blinds if blinds is not None else BlindStructure()

        # Betting state
        self.current_bet_amount: int = 0
        self.last_raiser_index: int = -1
        self.players_acted_in_round: int = 0
        self.last_raise_size: int = 0  # most recent legal raise increment

        # Vision flags
        self.card_detection_active: bool = False

        # Signals
        self.on_phase_change = Signal()
        self.on_pot_change = Signal()
        self.on_card_detection_required = Signal()
        self.on_player_action = Signal()
        self.on_turn_change = Signal()
        self.on_action_rejected = Signal()          # str (reason)
        self.on_hand_started = Signal()             # () — emitted after reset, before blinds
        self.on_dealing_required = Signal()         # (List[int] seats, int cards_per_seat)
        self.on_community_flip_required = Signal()  # (int count)
        self.on_pot_collection_required = Signal()  # (int winner_seat)

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
        """Resets state for a new hand and posts blinds."""
        if len(self.players) < 3:
            self.on_action_rejected.emit("Need at least 3 players to start a hand")
            return

        self.deck.reset()
        self.deck.shuffle()
        self.community_cards = []
        self.pot = ChipStack()
        self.phase = GamePhase.PRE_FLOP
        self.current_bet_amount = 0
        self.last_raiser_index = -1
        self.last_raise_size = 0
        self.players_acted_in_round = 0

        # Reset all players for the new hand
        for player in self.players:
            player.reset_for_new_hand()

        # Deal 2 hole cards to every non-sitting-out player
        for player in self.players:
            if player.status != PlayerStatus.SITTING_OUT:
                player.set_hole_cards(self.deck.deal(2))

        # Rotate dealer button (skip sitting-out seats)
        self.dealer_index = self._next_active_seat(self.dealer_index)

        # Emit on_hand_started AFTER reset/deal but BEFORE posting blinds
        self.on_hand_started.emit()

        # Post blinds (small blind = seat after dealer; big blind = seat after SB)
        sb_index = self._next_active_seat(self.dealer_index)
        bb_index = self._next_active_seat(sb_index)
        self._post_blind(sb_index, ActionType.POST_SB, self.blinds.small_blind)
        self._post_blind(bb_index, ActionType.POST_BB, self.blinds.big_blind)

        # Tell the arm to deal hole cards. Seats in turn order starting after the
        # button (SB first, then BB, then UTG, ...). Each active seat receives 2.
        dealing_order: List[int] = []
        seat_cursor = self.dealer_index
        for _ in range(len(self.players)):
            seat_cursor = self._next_active_seat(seat_cursor)
            if self.players[seat_cursor].status != PlayerStatus.SITTING_OUT:
                dealing_order.append(self.players[seat_cursor].seat)
        self.on_dealing_required.emit(dealing_order, 2)

        # First to act preflop = seat after BB (UTG)
        self.current_player_index = self._next_active_seat(bb_index)

        # BB is the implicit "raiser" preflop so action ends when it returns to BB
        self.current_bet_amount = self.blinds.big_blind
        self.last_raiser_index = bb_index
        self.last_raise_size = self.blinds.big_blind

        self.on_phase_change.emit(self.phase)
        self.on_turn_change.emit(self.current_player)

    def _next_active_seat(self, from_index: int) -> int:
        """Returns the next seat index (modulo player count) that is not SITTING_OUT."""
        n = len(self.players)
        for offset in range(1, n + 1):
            i = (from_index + offset) % n
            if self.players[i].status != PlayerStatus.SITTING_OUT:
                return i
        return from_index  # fallback if everyone is sitting out

    def _post_blind(self, player_index: int, blind_type: ActionType, amount: int):
        """Forces a blind bet from the player at player_index without turn validation."""
        player = self.players[player_index]
        actual_stack = player.bet(amount)
        player.total_committed += actual_stack.total
        self.update_pot(actual_stack)
        # Emit as an action so vision_controller and log see it
        action = Action(player.player_id, blind_type, actual_stack.total)
        self.on_player_action.emit(action)

    def next_turn(self):
        """Advances turn to the next active player."""
        if not self.get_eligible_players():
            return

        n = len(self.players)
        # Capture the index AFTER the first increment so the safety break only
        # fires when we have looped through every seat without finding an active
        # one — not on the first iteration as the old guard did.
        for offset in range(1, n + 1):
            candidate = (self.current_player_index + offset) % n
            if self.players[candidate].is_active():
                self.current_player_index = candidate
                self.on_turn_change.emit(self.current_player)
                return
        # No active player found

    def deal_community_cards(self, count: int):
        """Deals cards to the board and triggers detection."""
        new_cards = self.deck.deal(count)
        self.community_cards.extend(new_cards)
        self.on_card_detection_required.emit(new_cards)

    def update_pot(self, stack: ChipStack):
        """
        Adds a ChipStack to the pot.
        Args:
            stack: ChipStack to add (preserves chip colour composition).
        """
        self.pot.add_stack(stack)
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
            self.on_action_rejected.emit(f"Player ID {action.player_id} not found")
            return

        if player != self.current_player:
            self.on_action_rejected.emit(f"It is not {player.name}'s turn")
            return

        action_type = action.action_type
        amount = action.amount
        
        # Validate and Execute Action
        if action_type == ActionType.FOLD:
            player.fold()
            in_hand = self.get_players_in_hand()
            if len(in_hand) == 1:
                # Fold-out win: skip remaining streets and award directly
                self.players_acted_in_round += 1
                self.on_player_action.emit(action)
                self._jump_to_showdown()
                return

        elif action_type == ActionType.CHECK:
            if player.current_bet < self.current_bet_amount:
                self.on_action_rejected.emit("Cannot check, must call")
                return
            pass # Check is valid
            
        elif action_type == ActionType.CALL:
            to_call = self.current_bet_amount - player.current_bet
            actual_stack = player.bet(to_call)
            player.total_committed += actual_stack.total
            self.update_pot(actual_stack)
            action.amount = actual_stack.total

        elif action_type in (ActionType.RAISE, ActionType.BET, ActionType.ALL_IN):
            if action_type == ActionType.ALL_IN:
                # ALL_IN → translate to raise of remaining stack
                to_add = player.stack.total
                if to_add <= 0:
                    self.on_action_rejected.emit("Player has no chips to go all-in")
                    return
            else:
                if amount <= 0:
                    self.on_action_rejected.emit(f"Raise amount {amount} must be positive")
                    return
                new_total_bet = self.current_bet_amount + amount
                to_add = new_total_bet - player.current_bet

            actual_stack = player.bet(to_add)
            player.total_committed += actual_stack.total
            self.update_pot(actual_stack)
            action.amount = actual_stack.total

            # Reopen action only if the new total bet exceeds (current_bet + last_raise_size).
            # Short all-ins below that threshold do not reset players_acted_in_round.
            new_total = player.current_bet
            reopen_threshold = self.current_bet_amount + self.last_raise_size
            if new_total > self.current_bet_amount:
                if new_total >= reopen_threshold:
                    self.last_raise_size = new_total - self.current_bet_amount
                    self.last_raiser_index = self.current_player_index
                    self.players_acted_in_round = 0
                self.current_bet_amount = new_total


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
            # Cards exist in community_cards; arm can now flip them
            self.on_community_flip_required.emit(cards_to_deal)

        if self.phase == GamePhase.SHOWDOWN:
            self._award_pot_at_showdown()
            return

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

    def _jump_to_showdown(self):
        """Skip remaining streets and go directly to SHOWDOWN."""
        self.phase = GamePhase.SHOWDOWN
        self.on_phase_change.emit(self.phase)
        self._award_pot_at_showdown()

    def _award_pot_at_showdown(self):
        """Evaluate hands, build side pots from contribution tally, award winners.

        Side pots: sort distinct commit levels ascending. Each level forms a pot of
        (level - prev_level) * count_of_players_at_or_above_level, eligible to
        players still in hand (not folded) at that level. Top phevaluator score
        among eligibles for each pot wins; ties split. Integer remainders go to
        the player closest left of the dealer button.
        """
        from poker.evaluator import best_hand_score

        in_hand_players = [p for p in self.players if p.status != PlayerStatus.FOLDED]
        if len(in_hand_players) == 1:
            # Fold-out win — award entire pot directly
            winner = in_hand_players[0]
            winner.collect_winnings(self.pot.copy())
            self.pot = ChipStack()
            self.on_pot_change.emit(0)
            self.on_pot_collection_required.emit(winner.seat)
            print(f"GameState: Hand awarded to {winner.name} (fold-out)")
            return

        # Score every player still in hand. Need community cards present.
        if len(self.community_cards) < 5:
            return

        scores = {p.player_id: best_hand_score(p.hole_cards, self.community_cards)
                  for p in in_hand_players}

        # Build side pot levels from distinct total_committed values among ALL committed players
        commit_levels = sorted(set(p.total_committed for p in self.players if p.total_committed > 0))
        prev_level = 0

        for level in commit_levels:
            increment = level - prev_level
            if increment <= 0:
                continue
            contributors = [p for p in self.players if p.total_committed >= level]
            pot_size = increment * len(contributors)
            if pot_size == 0:
                continue

            # Eligibles for this side pot = contributors who are still in hand
            eligibles = [p for p in contributors if p.status != PlayerStatus.FOLDED]
            if not eligibles:
                prev_level = level
                continue

            # Determine winner(s) by lowest score
            best = min(scores[p.player_id] for p in eligibles)
            winners = [p for p in eligibles if scores[p.player_id] == best]
            share = pot_size // len(winners)
            remainder = pot_size - (share * len(winners))

            # Award shares
            for w in winners:
                w.collect_winnings(ChipStack.from_total(share))

            # Distribute remainder starting from seat closest left of button
            if remainder > 0:
                ordered = sorted(winners, key=lambda p: (p.seat - self.dealer_index) % len(self.players))
                for i in range(remainder):
                    ordered[i % len(ordered)].collect_winnings(ChipStack.from_total(1))

            # Tell the arm to push this pot's chips to each winner (seat order)
            for w in sorted(winners, key=lambda p: p.seat):
                self.on_pot_collection_required.emit(w.seat)

            prev_level = level

        # Clear pot
        self.pot = ChipStack()
        self.on_pot_change.emit(0)
