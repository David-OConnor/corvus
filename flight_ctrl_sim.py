import numpy as np
import matplotlib.pyplot as plt

TAU = 2. * np.pi

N = 10_000

DT = 1. / 6_666.666

# this many sim ticks per DT.
SIM_RATIO = 1

ctrl_effectiveness = 1

p_ω = 10.
time_to_correction_p_ω = 0.1
time_to_correction_p_θ = 0.5
max_ω_dot = 10.

# The change we need to effect given the initial conditions below.
θ_target = 10.

θ = np.zeros(N)
ω = np.zeros(N)
ω_dot = np.zeros(N)

θ[0] = TAU/5
ω[0] = 0
#ω_dot[0] = 0

ω_dot_current = 0;

for i in range(1, N-1):
    # if i % SIM_RATIO == 0:
    dθ = θ_target - θ[i]
    
    ω_target = dθ * p_ω

    # dist = 10
    dω = ω_target - ω[i]

    print(f"θ[i]: {θ[i]}, dθ: {dθ}, ω_target: {ω_target}, \
         dω: {dω}, ω_dot_current: {ω_dot_current}")

    time_to_correction = time_to_correction_p_ω * abs(dω) + \
        time_to_correction_p_θ * abs(dθ)

    ω_dot_target = dω / time_to_correction

    if ω_dot_target > max_ω_dot:
        ω_dot_target = max_ω_dot
    
    # let ctrl_effectiveness = ω_dot / ctrl_cmd_prev;

    # ω_dot[i] = ω_dot_target / ctrl_effectiveness

    ω_dot_current = ω_dot_target / ctrl_effectiveness

    # todo: More sophisticated integration method.
    ω[i + 1] = ω[i-1] + ω_dot_current * DT / SIM_RATIO
    θ[i + 1] = θ[i-1] + ω[i] * DT / SIM_RATIO
    


plt.plot(θ)
plt.show()
# plt.plot(ω)
# plt.show()

# plt.plot(ω_dot)


