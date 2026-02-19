import os

# Configuration
ASSETS_DIR = 'assets'
RANKS = ['2', '3', '4', '5', '6', '7', '8', '9', '10', 'J', 'Q', 'K', 'A']

# Colors from examples
COLOR_RED = "#E63946"
COLOR_BLACK = "#1A1A1A"

SUITS = {
    'C': {
        'name': 'Clubs',
        'color': COLOR_BLACK,
        'path': "M12 2a4 4 0 0 0-4 4c0 .8.2 1.5.6 2.1C5.7 8.5 3.5 10.5 3.5 13.5a5 5 0 0 0 5 5c.6 0 1.2-.1 1.8-.3l-1.3 3.8h6l-1.3-3.8c.6.2 1.2.3 1.8.3a5 5 0 0 0 5-5c0-3-2.2-5-5.1-5.4.4-.6.6-1.3.6-2.1a4 4 0 0 0-4-4Z",
        'text_y': 15.5
    },
    'D': {
        'name': 'Diamonds',
        'color': COLOR_RED,
        'path': "m12 2 9 10-9 10-9-10z",
        'text_y': 15.5
    },
    'H': {
        'name': 'Hearts',
        'color': COLOR_RED,
        'path': "M12 21.35l-1.45-1.32C5.4 15.36 2 12.28 2 8.5 2 5.42 4.42 3 7.5 3c1.74 0 3.41.81 4.5 2.09C13.09 3.81 14.76 3 16.5 3 19.58 3 22 5.42 22 8.5c0 3.78-3.4 6.86-8.55 11.54L12 21.35z",
        'text_y': 14.5
    },
    'S': {
        'name': 'Spades',
        'color': COLOR_BLACK,
        'path': "M12 2C9 2 4 8 4 12c0 2.21 1.79 4 4 4 .83 0 1.59-.25 2.24-.68L9 22h6l-1.24-6.68c.65.43 1.41.68 2.24.68 2.21 0 4-1.79 4-4 0-4-5-10-8-10z",
        'text_y': 12.3
    }
}

def get_font_props(rank):
    # Adjust font size and letter spacing for '10'
    if rank == '10':
        return '6.5', '-1', '12' # smaller font for 2 digits
    else:
        return '9', '0', '12' # standard size

def create_icon_svg(rank, suit_code, filename):
    suit_data = SUITS[suit_code]
    color = suit_data['color']
    path_d = suit_data['path']
    base_text_y = suit_data['text_y']
    
    # Calculate unique mask ID to avoid conflicts if inlined later
    mask_id = f"{suit_code.lower()}-{rank.lower()}-mask"
    
    font_size, letter_spacing, text_x = get_font_props(rank)
    
    # SVG Template
    svg_content = f"""<svg width="200" height="200" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink">
<defs>
<mask id="{mask_id}">
<rect width="24" height="24" fill="white" />
<text x="{text_x}" y="{base_text_y}" font-family="Arial, sans-serif" font-weight="900" font-size="{font_size}" letter-spacing="{letter_spacing}" text-anchor="middle" fill="black">{rank}</text>
</mask>
</defs>
<path d="{path_d}" fill="{color}" mask="url(#{mask_id})" />
</svg>"""
    
    with open(os.path.join(ASSETS_DIR, filename), 'w') as f:
        f.write(svg_content)

if __name__ == "__main__":
    if not os.path.exists(ASSETS_DIR):
        os.makedirs(ASSETS_DIR)
        
    for suit_code in SUITS:
        for rank in RANKS:
            filename = f"{rank}{suit_code}.svg"
            print(f"Generating {filename}...")
            create_icon_svg(rank, suit_code, filename)
            
    print("Done generating negative space icons.")
