import numpy as np
import matplotlib.pyplot as plt

TAU = 2. * np.pi

N = 50_000

# DT = 1. / 6_666.666  # todo
DT = 1./10.

# this many sim ticks per DT.
SIM_RATIO = 60

ctrl_effectiveness = 1

p_ω = 0.10
time_to_correction_p_ω = 0.1
time_to_correction_p_θ = 0.1
max_ω_dot = 10.

# The change we need to effect given the initial conditions below.
θ_target = 10.

θ = np.zeros(N)
ω = np.zeros(N)
ω_dot = np.zeros(N)

# θ[0] = TAU/5
# θ[1] = TAU/5
θ[0] = 0
ω[0] = 0


#ω_dot[0] = 0

ω_dot_current = 0;

for i in range(1, N-2):

    # todo: More sophisticated integration method.
    # ω[i] = ω[i-1] + ω_dot[i-1] * DT / SIM_RATIO
    ω[i] = ω[i-1] + ω_dot_current * DT / SIM_RATIO
    θ[i] = θ[i-1] + ω[i-1] * DT / SIM_RATIO
    
    if i % SIM_RATIO == 0:
        dθ = θ_target - θ[i]
        
        ω_target = dθ * p_ω

        dω = ω_target - ω[i]

        time_to_correction = time_to_correction_p_ω * abs(dω) + \
            time_to_correction_p_θ * abs(dθ)

        ω_dot_target = dω / time_to_correction

        # print(f"θ[i]: {θ[i]}, dθ: {dθ}, ω_target: {ω[i]}, \
        #      dω: {dω}, ω_dot_current: {ω_dot_current}\n")

        # print(f"ttc: {time_to_correction}, wdt: {ω_dot_target}\n")

        # todo: Add this back with signed abs.
        # if abs(ω_dot_target) > max_ω_dot:
            # ω_dot_target = max_ω_dot
        # 
        # let ctrl_effectiveness = ω_dot / ctrl_cmd_prev;

        # ω_dot[i] = ω_dot_target / ctrl_effectiveness

        ω_dot_current = ω_dot_target / ctrl_effectiveness
        # print(f"ω_dot_current: {ω_dot_current}")
            

plt.plot(θ)
plt.show()
plt.plot(ω)
plt.show()

# plt.plot(ω_dot)


