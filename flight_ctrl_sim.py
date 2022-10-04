import numpy as np
import matplotlib.pyplot as plt

from numpy import sqrt

TAU = 2. * np.pi

N = 50_000

# DT = 1. / 6_666.666  # todo
DT = 1./10.

# this many sim ticks per DT.
SIM_RATIO = 200

ctrl_effectiveness = 1

p_ω = 0.30
time_to_correction_p_ω = 0.2
time_to_correction_p_θ = 0.5
max_ω_dot = 10.

drag_coeff_ctrls = 0.01

# The change we need to effect given the initial conditions below.
θ_target = 10.

θ = np.zeros(N)
ω = np.zeros(N)
ω_dot = np.zeros(N)

# θ[0] = TAU/5
# θ[1] = TAU/5
θ[0] = 0
ω[0] = 0

drag_coeff_sim = 0.01

ω_dot_current = 0;

# temp: fixing jerk
omega_dot_dot_held = 0

def run():
    for i in range(1, N-2):

        # todo: More sophisticated integration method.
        # ω[i] = ω[i-1] + ω_dot[i-1] * DT / SIM_RATIO
        
        # todo: Incorporate drag in sim and control code

        ω[i] = ω[i-1] + ω_dot_current * DT / SIM_RATIO
        θ[i] = θ[i-1] + ω[i-1] * DT / SIM_RATIO
        ω_dot[i] = ω_dot_current
        
        if i % SIM_RATIO == 0:
            dθ = θ_target - θ[i]

            # ω_target = dθ * p_ω

            # dω = ω_target - ω[i]

            # time_to_correction = time_to_correction_p_ω * abs(dω) + \
            #     time_to_correction_p_θ * abs(dθ)

            time_to_correction = time_to_correction_p_θ * abs(dθ)

            ω_dot_start = -ω[i] * 2. / time_to_correction**2

            ω_dot_dot = 6./ time_to_correction**3 * \
                (ω[i] - dθ - ω[i] * time_to_correction)

            # ω_dot_target = dω / time_to_correction

            # print(f"θ[i]: {θ[i]}, dθ: {dθ}, ω_target: {ω[i]}, \
            #      dω: {dω}, ω_dot_current: {ω_dot_current}\n")

            # print(f"ttc: {time_to_correction}, wdt: {ω_dot_target}\n")

            # todo: Add this back with signed abs.
            # if abs(ω_dot_target) > max_ω_dot:
                # ω_dot_target = max_ω_dot
            # 
            # let ctrl_effectiveness = ω_dot / ctrl_cmd_prev;

            # ω_dot[i] = ω_dot_target / ctrl_effectiveness

            # todo: sign?
            ω_dot_tgt = -ω_dot_start - ω_dot_dot * DT


            ω_dot_current = ω_dot_tgt / ctrl_effectiveness
            # print(f"ω_dot_current: {ω_dot_current}")
            

    plt.plot(θ)
    plt.show()
    plt.plot(ω)
    plt.show()
    plt.plot(ω_dot)
    plt.show()


def plot_analytic():
    t = 1.

    θ_0 = 1
    # ω_0 = 0

    # for ω_0 in [-2., -1., -0.5, 0., 0.5,2 1., 2.]:
    for θ_0 in [0.5, 1.]:
        for ω_0 in [0., 1., 2.]:
            c = θ_0 + t * ω_0  # Don't duplicate computation
            ω_dot_0 = -6. * c / t**2

            # These 2 are equivalent
            ω_dot_dot = 12. * c / t**3
            # ω_dot_dot = -2. * ω_dot_0 / t


            n = 15_000
            dt = 1e-4

            θ = np.zeros(n)
            ω = np.zeros(n)
            j = np.zeros(n)

            for i in range(n):
                t_ = i * dt

                θ[i] = θ_0 + ω_0 * t_ + 1/2 * ω_dot_0 * t_**2 + 1/6 * ω_dot_dot * t_**3

            plt.plot(θ)
    plt.show()

plot_analytic()
# run()




