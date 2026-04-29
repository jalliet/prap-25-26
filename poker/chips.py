from enum import Enum
from typing import Dict


class ChipColour(Enum):
    """
    Represents standard poker chip colours with their associated monetary values.

    This enumeration serves as the ground truth for chip denomination in the system.
    """
    RED = 1
    BLUE = 5
    WHITE = 20

    @property
    def code(self) -> str:
        """Returns the short code for the chip colour (e.g., 'R', 'B')."""
        codes = {
            1: "R",
            5: "B",
            20: "W",
        }
        return codes[self.value]

    @classmethod
    def from_code(cls, code: str) -> 'ChipColour':
        """
        Factory method to create a ChipColour from a short code.
        Args:
            code: The short code string (e.g., 'R', 'B', 'W').
        Returns:
            The corresponding ChipColour enum member.
        """
        code_map = {
            "R": cls.RED,
            "B": cls.BLUE,
            "W": cls.WHITE,
        }
        colour = code_map.get(code.upper())
        if colour is None:
            raise ValueError(f"Invalid chip colour code: {code}")
        return colour


class ChipStack:
    """
    Represents a physical or virtual stack of poker chips.
    
    This data structure maintains the count of each chip denomination and provides
    methods for arithmetic operations (add, remove) and value calculation.
    """
    
    def __init__(self, chips: Dict[ChipColour, int] = None):
        """
        Initialise a ChipStack with a dictionary of chips.
        
        Args:
            chips: Dictionary mapping ChipColour to count. Defaults to empty stack.
        """
        self._chips: Dict[ChipColour, int] = {colour: 0 for colour in ChipColour}
        if chips:
            for colour, count in chips.items():
                if count < 0:
                    raise ValueError(f"Chip count cannot be negative: {colour.name}={count}")
                self._chips[colour] = count
    
    @property
    def total(self) -> int:
        """Calculates the total monetary value of all chips in the stack."""
        return sum(colour.value * count for colour, count in self._chips.items())
    
    def count(self, colour: ChipColour) -> int:
        """Returns the count of chips for a specific colour."""
        return self._chips[colour]
    
    def add(self, colour: ChipColour, n: int) -> None:
        """Adds n chips of a specific colour to the stack."""
        if n < 0:
            raise ValueError(f"Cannot add negative chips: {n}")
        self._chips[colour] += n
    
    def remove(self, colour: ChipColour, n: int) -> None:
        """Removes n chips of a specific colour from the stack."""
        if n < 0:
            raise ValueError(f"Cannot remove negative chips: {n}")
        if self._chips[colour] < n:
            raise ValueError(
                f"Cannot remove {n} {colour.name} chips, only have {self._chips[colour]}"
            )
        self._chips[colour] -= n
    
    def add_stack(self, other: 'ChipStack') -> None:
        """Merges another chip stack into this one."""
        for colour in ChipColour:
            self._chips[colour] += other._chips[colour]
    
    def can_remove_stack(self, other: 'ChipStack') -> bool:
        """Checks if this stack contains enough chips of each colour to remove the other stack."""
        for colour in ChipColour:
            if self._chips[colour] < other._chips[colour]:
                return False
        return True
    
    def remove_stack(self, other: 'ChipStack') -> None:
        """Removes the chips present in the other stack from this one."""
        if not self.can_remove_stack(other):
            raise ValueError("Insufficient chips to remove stack")
        for colour in ChipColour:
            self._chips[colour] -= other._chips[colour]
    
    def is_empty(self) -> bool:
        """Returns True if the stack has zero total value."""
        return self.total == 0
    
    def copy(self) -> 'ChipStack':
        """Creates a deep copy of this chip stack."""
        return ChipStack(self._chips.copy())
    
    def breakdown(self) -> Dict[ChipColour, int]:
        """Returns a dictionary of chips present in the stack (count > 0)."""
        return {colour: count for colour, count in self._chips.items() if count > 0}
    
    def __str__(self) -> str:
        """
        Returns a human-readable string representation of the stack.
        Format: "W:5(5) R:2(10) Total 15"
        """
        parts = []
        for colour in ChipColour:
            count = self._chips[colour]
            if count > 0:
                value = count * colour.value
                parts.append(f"{colour.code}:{count}({value})")
        if not parts:
            return "Empty (0)"
        return " ".join(parts) + f" Total {self.total}"
    
    def __repr__(self) -> str:
        """
        Returns an unambiguous string representation for debugging.
        Format: ChipStack(total=X, breakdown={...})
        """
        return f"ChipStack(total={self.total}, breakdown={self.breakdown()})"
    
    def __eq__(self, other) -> bool:
        if not isinstance(other, ChipStack):
            return False
        return self._chips == other._chips
    
    @classmethod
    def from_total(cls, total: int) -> 'ChipStack':
        """
        Factory method to create a ChipStack from a total integer value.
        Uses a greedy algorithm to approximate the optimal breakdown (largest denominations first).
        
        Args:
            total: The total monetary value to represent.
        Returns:
            A ChipStack instance with the calculated chip counts.
        """
        if total < 0:
            raise ValueError(f"Total cannot be negative: {total}")
        
        remaining = total
        chips = {}
        
        # Sort by value descending for greedy breakdown
        for colour in sorted(ChipColour, key=lambda c: c.value, reverse=True):
            count = remaining // colour.value
            if count > 0:
                chips[colour] = count
                remaining -= count * colour.value
        
        return cls(chips)


def make_stack(**kwargs) -> ChipStack:
    """
    Convenience factory function to create a chip stack from keyword arguments.
    
    Args:
        **kwargs: Key is the chip code (e.g., 'R', 'W'), value is the count.
        
    Example:
        make_stack(R=3, W=5) -> ChipStack with 3 Red and 5 White chips.
    """
    chips = {}
    for code, count in kwargs.items():
        colour = ChipColour.from_code(code)
        chips[colour] = count
    return ChipStack(chips)
