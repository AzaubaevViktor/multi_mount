import math

def compute_pole_offset(d: float, k: float, HA_deg: float, dec_deg: float):
    """
    d      - дрейф по Dec ["/с], положительный = на север
    k      - коэффициент RA относительно сидерической (1.0 = точно)
    HA_deg - часовой угол звезды [градусы], запад положительный
    dec_deg- склонение звезды [градусы]
    
    Возвращает (eps_N, eps_E) в угловых секундах.
    eps_N > 0 — полюс севернее оси монтировки
    eps_E > 0 — полюс восточнее оси монтировки
    """
    omega = 15.0  # "/с, сидерическая скорость

    HA  = math.radians(HA_deg)
    dec = math.radians(dec_deg)

    tan_dec = math.tan(dec)
    if abs(tan_dec) < 1e-6:
        raise ValueError("Склонение слишком близко к 0° — уравнение по RA вырождено")

    # Нормированные правые части
    rhs_dec = d / omega          # из уравнения по Dec
    rhs_ra  = (k - 1) / tan_dec  # из уравнения по RA

    eps_N = rhs_dec * math.cos(HA) + rhs_ra * math.sin(HA)
    eps_E = -rhs_dec * math.sin(HA) + rhs_ra * math.cos(HA)

    return eps_N, eps_E


def main():
    print("=== Вычисление смещения полюса ===\n")
    d       = float(input("Дрейф по Dec [\"/ с] (+ север, - юг): "))
    k       = float(input("Коэффициент RA (например 1.0003): "))
    HA_deg  = float(input("Часовой угол звезды [градусы] (+ запад): "))
    dec_deg = float(input("Склонение звезды [градусы]: "))

    eps_N, eps_E = compute_pole_offset(d, k, HA_deg, dec_deg)

    print(f"\nСмещение полюса от оси монтировки:")
    print(f"  ε_N = {eps_N:+.2f}\"  ({'полюс севернее' if eps_N > 0 else 'полюс южнее'} оси)")
    print(f"  ε_E = {eps_E:+.2f}\"  ({'полюс восточнее' if eps_E > 0 else 'полюс западнее'} оси)")
    print(f"\nПолное смещение: {math.hypot(eps_N, eps_E):.2f}\"")

    az_err = math.degrees(math.atan2(eps_E, eps_N))
    print(f"Направление смещения (от севера на восток): {az_err:.1f}°")
    print(f"\nЧтобы совместить ось с полюсом:")
    print(f"  Высоту {'увеличить' if eps_N < 0 else 'уменьшить'} на {abs(eps_N):.2f}\"")
    print(f"  Азимут {'повернуть на восток' if eps_E < 0 else 'повернуть на запад'} на {abs(eps_E):.2f}\"")


if __name__ == "__main__":
    main()