from typing import List, Tuple
from poker.player import Player


# Position labels by player count (clockwise from button)
POSITIONS = {
    2: ["BTN", "BB"],  # BTN is also SB
    3: ["BTN", "SB", "BB"],
    4: ["BTN", "SB", "BB", "UTG"],
    5: ["BTN", "SB", "BB", "UTG", "CO"],
    6: ["BTN", "SB", "BB", "UTG", "MP", "CO"]
}


def assign_positions(
    players: List[Player],
    button_seat: int
) -> Tuple[int, int]:
    """
    Assign position labels to players based on button location.
    Args:
        players: List of active players (must have valid seat attributes)
        button_seat: Seat number of the dealer button
    Returns:
        Tuple of (small_blind_seat, big_blind_seat)
    Raises:
        ValueError: If player count is not 2-6
    """
    player_count = len(players)
    
    if player_count < 2 or player_count > 6:
        raise ValueError(f"Player count must be 2-6, got {player_count}")
    
    # Sort players by seat, starting from button
    seats = sorted([p.seat for p in players])
    
    # Find button index in sorted seats
    if button_seat not in seats:
        raise ValueError(f"Button seat {button_seat} not found in player seats")
    
    button_idx = seats.index(button_seat)
    
    # Reorder seats starting from button (clockwise)
    ordered_seats = seats[button_idx:] + seats[:button_idx]
    
    # Get position labels for this player count
    labels = POSITIONS[player_count]
    
    # Create seat -> label mapping, label players
    seat_to_label = {seat: labels[i] for i, seat in enumerate(ordered_seats)}
    for player in players:
        player.position_label = seat_to_label[player.seat]
    
    # Determine SB & BB seats
    if player_count == 2: # BTN is SB
        sb_seat = ordered_seats[0]  # BTN/SB
        bb_seat = ordered_seats[1]  # BB
    else:
        sb_seat = ordered_seats[1]  # SB after BTN
        bb_seat = ordered_seats[2]  # BB after SB
    
    return sb_seat, bb_seat


def get_preflop_order(players: List[Player]) -> List[Player]:
    """
    Get players in preflop acting order (UTG first, BB last).
    Args:
        players: List of players with position_label assigned
    Returns:
        List of players in preflop acting order
    """
    player_count = len(players)
    labels = POSITIONS[player_count]
    
    # Preflop order: after BB, then SB, then UTG
    # For 6 players: UTG, MP, CO, BTN, SB, BB
    if player_count == 2:
        # Heads-up: SB/BTN acts first, BB acts second
        order = ["BTN", "BB"]
    else:
        # Start from after BB, wrap around
        bb_idx = labels.index("BB")
        order = labels[bb_idx + 1:] + labels[:bb_idx + 1]
    
    label_to_player = {p.position_label: p for p in players}
    return [label_to_player[label] for label in order]


def get_postflop_order(players: List[Player]) -> List[Player]:
    """
    Get players in postflop acting order (SB first, BTN last).
    Only includes players still in hand.
    Args:
        players: List of players with position_label assigned
    Returns:
        List of active players in postflop acting order
    """
    player_count = len(players)
    labels = POSITIONS[player_count]
    
    if player_count == 2:
        # Heads-up: BB acts first postflop
        order = ["BB", "BTN"]
    else:
        # SB first, then BB, then others, BTN last
        btn_idx = labels.index("BTN")
        order = labels[btn_idx + 1:] + labels[:btn_idx + 1]
    
    label_to_player = {p.position_label: p for p in players}
    return [label_to_player[label] for label in order if label in label_to_player]
