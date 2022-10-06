import numpy as np
import matplotlib.pyplot as plt

from numpy import sqrt

TAU = 2. * np.pi

N = 50_000

# DT = 1. / 6_666.666  # todo
DT = 1./10.

# this many sim ticks per DT.
SIM_RATIO = 100

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
θ[0] = 1
ω[0] = 0
ω_dot[0] = 0

drag_coeff_sim = 0.01

# temp: fixing jerk
omega_dot_dot_held = 0

def run():
    ω_dot_current = 0

    for i in range(1, N-2):

        # todo: More sophisticated integration method.
        # ω[i] = ω[i-1] + ω_dot[i-1] * DT / SIM_RATIO
        
        # todo: Incorporate drag in sim and control code

        ω[i] = ω[i-1] + ω_dot_current * DT / SIM_RATIO
        θ[i] = θ[i-1] + ω[i-1] * DT / SIM_RATIO
        ω_dot[i] = ω_dot_current
        
        if i % SIM_RATIO == 0:
            dθ = θ_target - θ[i]

            # ttc = time_to_correction_p_ω * abs(dω) + \
            #     time_to_correction_p_θ * abs(dθ)

            ttc = time_to_correction_p_θ * abs(dθ)

            ω_dot_0 = -(6.*θ[i] + 4.*ttc*ω[i]) / ttc**2

            ω_dot_dot = 6. * (2.*θ[i] + ttc*ω[i]) / ttc**3

            ω_dot_tgt = -ω_dot_0 - ω_dot_dot * DT

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

    for θ_0 in [0.5, 1.]:
        for ω_0 in [0., 1., 2.]:
            ω_dot_0 = -(6.*θ_0 + 4.*t*ω_0) / t**2

            ω_dot_dot = 6. * (2.*θ_0 + t*ω_0) / t**3

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

# plot_analytic()
run()




