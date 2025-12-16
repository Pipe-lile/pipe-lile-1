"""
Cuenta regresiva desde 10 usando un ciclo while.
"""


def countdown(start: int = 10) -> None:
    """Imprime una cuenta regresiva desde ``start`` hasta 1."""
    current = start
    while current > 0:
        print(current)
        current -= 1


if __name__ == "__main__":
    countdown()
