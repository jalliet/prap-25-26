### **Plan: Create Kivy Dashboard UI Skeleton**

1.  **Project Structure**
    *   Create `gui/` directory.
    *   Create `requirements.txt` (including `kivy`).

2.  **Dashboard Layout (`gui/dashboard.kv`)**
    *   **Root Layout**: Horizontal split (30% Left, 70% Right).
    *   **Left Panel (Game Tracking)**:
        *   **Community Cards**: Placeholder row (visual slots for cards).
        *   **Pot Display**: Large label.
        *   **Player Panel**: Scrollable list with `PlayerWidget` (Name, Stack, Cards, Status).
        *   **Log**: Text area for game history.
    *   **Right Panel (Camera Feed)**:
        *   Large placeholder widget (dark background, labeled "Live Camera Feed") to reserve space for the future video stream.

3.  **Application Entry Point (`gui/dashboard.py`)**
    *   Implement `PokerDashboardApp` class.
    *   Initialize the app with the KV layout.
    *   Populate the list with static dummy players (e.g., "Player 1", "Player 2") to demonstrate the interface structure without backend logic.

### **Verification**
*   Run `python gui/dashboard.py` to confirm the interface loads correctly and the layout matches the intended design.
