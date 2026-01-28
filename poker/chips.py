from enum import Enum
from typing import Dict


class ChipColour(Enum):
    """Chip colors with their values."""
    WHITE = 1
    RED = 5
    BLUE = 25
    BLACK = 50
    
    @property
    def code(self) -> str:
        codes = {
            1: "W",
            5: "R",
            25: "B",
            50: "BK"
        }
        return codes[self.value]
    
    @classmethod
    def from_code(cls, code: str) -> 'ChipColour':
        """Create ChipColour from code (W, R, B, BK)."""
        code_map = {
            "W": cls.WHITE,
            "R": cls.RED,
            "B": cls.BLUE,
            "BK": cls.BLACK
        }
        color = code_map.get(code.upper())
        if color is None:
            raise ValueError(f"Invalid chip color code: {code}")
        return color


class ChipStack:
    """
    Represents stack of poker chips with color breakdown.
    Track counts of each chip color, use operations for adding, subtracting, and calculating total value.
    """
    
    def __init__(self, chips: Dict[ChipColour, int] = None):
        """
        chips: Dict mapping ChipColour to count. Defaults to empty stack.
        """
        self._chips: Dict[ChipColour, int] = {color: 0 for color in ChipColour}
        if chips:
            for color, count in chips.items():
                if count < 0:
                    raise ValueError(f"Chip count cannot be negative: {color.name}={count}")
                self._chips[color] = count
    
    @property
    def total(self) -> int:
        """Calculate total value of all chips."""
        return sum(color.value * count for color, count in self._chips.items())
    
    def count(self, color: ChipColour) -> int:
        """Get count of chips of a specific color."""
        return self._chips[color]
    
    def add(self, color: ChipColour, n: int) -> None:
        """Add chips of a specific color."""
        if n < 0:
            raise ValueError(f"Cannot add negative chips: {n}")
        self._chips[color] += n
    
    def remove(self, color: ChipColour, n: int) -> None:
        """Remove chips of a specific color."""
        if n < 0:
            raise ValueError(f"Cannot remove negative chips: {n}")
        if self._chips[color] < n:
            raise ValueError(
                f"Cannot remove {n} {color.name} chips, only have {self._chips[color]}"
            )
        self._chips[color] -= n
    
    def add_stack(self, other: 'ChipStack') -> None:
        """Add another chip stack to this one."""
        for color in ChipColour:
            self._chips[color] += other._chips[color]
    
    def can_remove_stack(self, other: 'ChipStack') -> bool:
        """Check if we can remove another stack from this one."""
        for color in ChipColour:
            if self._chips[color] < other._chips[color]:
                return False
        return True
    
    def remove_stack(self, other: 'ChipStack') -> None:
        """Remove another chip stack from this one."""
        if not self.can_remove_stack(other):
            raise ValueError("Insufficient chips to remove stack")
        for color in ChipColour:
            self._chips[color] -= other._chips[color]
    
    def is_empty(self) -> bool:
        """Check if the stack has no chips."""
        return self.total == 0
    
    def copy(self) -> 'ChipStack':
        """Create a copy of this chip stack."""
        return ChipStack(self._chips.copy())
    
    def breakdown(self) -> Dict[ChipColour, int]:
        """Get the chip breakdown as a dict."""
        return {color: count for color, count in self._chips.items() if count > 0}
    
    def __str__(self) -> str:
        """Compact string representation: R:3(15) G:1(25) Total 40"""
        parts = []
        for color in ChipColour:
            count = self._chips[color]
            if count > 0:
                value = count * color.value
                parts.append(f"{color.code}:{count}({value})")
        if not parts:
            return "Empty (0)"
        return " ".join(parts) + f" Total {self.total}"
    
    def __repr__(self) -> str:
        return f"ChipStack({self.total})"
    
    def __eq__(self, other) -> bool:
        if not isinstance(other, ChipStack):
            return False
        return self._chips == other._chips
    
    @classmethod
    def from_total(cls, total: int) -> 'ChipStack':
        """
        Create chip stack from total stack value using largest denomination breakdown.
        Args:
            total: Total chip value to break down
        Returns:
            ChipStack with optimal breakdown
        """
        if total < 0:
            raise ValueError(f"Total cannot be negative: {total}")
        
        remaining = total
        chips = {}
        
        for color in sorted(ChipColour, key=lambda c: c.value, reverse=True):
            count = remaining // color.value
            if count > 0:
                chips[color] = count
                remaining -= count * color.value
        
        return cls(chips)


def make_stack(**kwargs) -> ChipStack:
    """
    Convenience function to create a chip stack.
    Examples:
        make_stack(R=3, G=1)  # 3 red + 1 green = 40
        make_stack(W=5)       # 5 white = 5
    """
    chips = {}
    for code, count in kwargs.items():
        color = ChipColour.from_code(code)
        chips[color] = count
    return ChipStack(chips)
