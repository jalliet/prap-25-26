import os

# Configuration
WIDTH = 200
HEIGHT = 300
ASSETS_DIR = 'assets'

RANKS = ['2', '3', '4', '5', '6', '7', '8', '9', '10', 'J', 'Q', 'K', 'A']
SUITS = {
    'C': {'name': 'Clubs', 'color': 'black', 'path': "M16,6 C13.5,6 11.5,8 11.5,10.5 C11.5,12.5 13,14 15,14.5 C13,14 10,12 8,12 C5,12 3,14.5 3,17.5 C3,20.5 5.5,23 8.5,23 C10.5,23 12,22 13,21 L13,26 L10,28 L22,28 L19,26 L19,21 C20,22 21.5,23 23.5,23 C26.5,23 29,20.5 29,17.5 C29,14.5 27,12 24,12 C22,12 19,14 17,14.5 C19,14 20.5,12.5 20.5,10.5 C20.5,8 18.5,6 16,6 Z"},
    'D': {'name': 'Diamonds', 'color': '#D40000', 'path': "M16,4 L26,16 L16,28 L6,16 Z"},
    'H': {'name': 'Hearts', 'color': '#D40000', 'path': "M16,28 C16,28 29,18 29,10 C29,5 25,2 21,2 C18,2 16,4 16,4 C16,4 14,2 11,2 C7,2 3,5 3,10 C3,18 16,28 16,28 Z"},
    'S': {'name': 'Spades', 'color': 'black', 'path': "M16,4 C16,4 10,12 6,18 C4,21 5,24 9,24 C11,24 13,23 14,21 L14,26 L10,28 L22,28 L18,26 L18,21 C19,23 21,24 23,24 C27,24 28,21 26,18 C22,12 16,4 16,4 Z"}
}

# Semantic Grid Definitions
COLS = {'L': 60, 'C': 100, 'R': 140}
ROWS = {
    'T': 60, 
    'MT': 105, 
    'C': 150, 
    'MB': 195, 
    'B': 240,
    '7_MID': 125,
    '10_UPPER': 82,
    '10_LOWER': 218
}

# Pip Configuration (Declarative Data)
# Format: (Column, Row, IsInverted)
PIP_PATTERNS = {
    '2': [('C', 'T', False), ('C', 'B', True)],
    '3': [('C', 'T', False), ('C', 'C', False), ('C', 'B', True)],
    '4': [('L', 'T', False), ('R', 'T', False), ('L', 'B', True), ('R', 'B', True)],
    '5': [('L', 'T', False), ('R', 'T', False), ('C', 'C', False), ('L', 'B', True), ('R', 'B', True)],
    '6': [('L', 'T', False), ('R', 'T', False), ('L', 'C', False), ('R', 'C', False), ('L', 'B', True), ('R', 'B', True)],
    '7': [('L', 'T', False), ('R', 'T', False), ('L', 'C', False), ('R', 'C', False), ('C', '7_MID', False), ('L', 'B', True), ('R', 'B', True)],
    '8': [('L', 'T', False), ('R', 'T', False), ('L', 'C', False), ('R', 'C', False), ('L', 'B', True), ('R', 'B', True), ('C', 'MT', False), ('C', 'MB', True)],
    '9': [('L', 'T', False), ('R', 'T', False), ('L', 'MT', False), ('R', 'MT', False), ('C', 'C', False), ('L', 'MB', True), ('R', 'MB', True), ('L', 'B', True), ('R', 'B', True)],
    '10': [('L', 'T', False), ('R', 'T', False), ('L', 'MT', False), ('R', 'MT', False), ('C', '10_UPPER', False), ('C', '10_LOWER', True), ('L', 'MB', True), ('R', 'MB', True), ('L', 'B', True), ('R', 'B', True)]
}

def get_pips(rank):
    """
    Returns list of (x, y, inverted) tuples based on declarative patterns.
    """
    if rank not in PIP_PATTERNS:
        return []
        
    layout = PIP_PATTERNS[rank]
    # Hydrate the semantic grid keys to actual coordinates
    return [(COLS[c], ROWS[r], inv) for (c, r, inv) in layout]

def create_svg(rank, suit_code, filename):
    suit_data = SUITS[suit_code]
    color = suit_data['color']
    path_d = suit_data['path']
    
    # SVG Header
    svg_content = [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {WIDTH} {HEIGHT}">',
        f'<rect x="2" y="2" width="{WIDTH-4}" height="{HEIGHT-4}" rx="15" ry="15" fill="white" stroke="black" stroke-width="2"/>'
    ]
    
    # Corner Value and Suit (Top Left)
    # Group for Top-Left
    svg_content.append('<g transform="translate(15, 20)">')
    # Rank text
    # Adjust font size for 10 to fit
    font_size = "28"
    letter_spacing = "0"
    if rank == '10':
        letter_spacing = "-2"
    
    svg_content.append(f'<text x="0" y="0" font-family="Arial, sans-serif" font-size="{font_size}" font-weight="bold" fill="{color}" letter-spacing="{letter_spacing}">{rank}</text>')
    # Suit icon below rank
    svg_content.append(f'<path d="{path_d}" transform="translate(0, 10) scale(0.6)" fill="{color}"/>')
    svg_content.append('</g>')
    
    # Corner Value and Suit (Bottom Right - Rotated)
    svg_content.append(f'<g transform="rotate(180, {WIDTH/2}, {HEIGHT/2}) translate(15, 20)">')
    svg_content.append(f'<text x="0" y="0" font-family="Arial, sans-serif" font-size="{font_size}" font-weight="bold" fill="{color}" letter-spacing="{letter_spacing}">{rank}</text>')
    svg_content.append(f'<path d="{path_d}" transform="translate(0, 10) scale(0.6)" fill="{color}"/>')
    svg_content.append('</g>')
    
    # Center Content
    if rank in ['J', 'Q', 'K']:
        # Face Card - Box with Large Letter
        svg_content.append(f'<rect x="50" y="50" width="100" height="200" fill="none" stroke="{color}" stroke-width="2"/>')
        svg_content.append(f'<text x="100" y="180" font-family="Arial, sans-serif" font-size="100" font-weight="bold" fill="{color}" text-anchor="middle">{rank}</text>')
        # Add inverted suit paths in corners of the box for flair
        svg_content.append(f'<path d="{path_d}" transform="translate(60, 60) scale(0.5)" fill="{color}"/>')
        svg_content.append(f'<path d="{path_d}" transform="translate(140, 240) scale(0.5) rotate(180, 16, 16)" fill="{color}"/>')
        
    elif rank == 'A':
        # Ace - Single Large Pip
        svg_content.append(f'<path d="{path_d}" transform="translate(68, 118) scale(2.0)" fill="{color}"/>')
        
    else:
        # Number Cards - Pips
        pips = get_pips(rank)
        for x, y, inverted in pips:
            transform = f"translate({x-16}, {y-16})"
            if inverted:
                # Rotate around center of the 32x32 icon (16,16)
                transform += " rotate(180, 16, 16)"
            svg_content.append(f'<path d="{path_d}" transform="{transform}" fill="{color}"/>')

    svg_content.append('</svg>')
    
    with open(os.path.join(ASSETS_DIR, filename), 'w') as f:
        f.write('\n'.join(svg_content))

if __name__ == "__main__":
    if not os.path.exists(ASSETS_DIR):
        os.makedirs(ASSETS_DIR)
        
    for suit_code in SUITS:
        for rank in RANKS:
            filename = f"{rank}{suit_code}.svg"
            print(f"Generating {filename}...")
            create_svg(rank, suit_code, filename)
    
    print("Done.")
