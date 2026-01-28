from kivy.app import App
from kivy.lang import Builder
from kivy.uix.boxlayout import BoxLayout
from kivy.core.window import Window

Window.size = (1200, 800)

class PokerDashboard(BoxLayout):
    pass

class PokerDashboardApp(App):
    def build(self):
        self.title = "PRAP 2025/2026 Dashboard"
        return PokerDashboard()

if __name__ == '__main__':
    PokerDashboardApp().run()
