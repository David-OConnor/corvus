import numpy as np
import matplotlib.pyplot as plt

from numpy import sqrt

TAU = 2. * np.pi

N = 50_000 * 10

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
# θ_target = 1.
# ω_target = 0.

θ = np.zeros(N)
ω = np.zeros(N)
ω_dot = np.zeros(N)

θ[0] = 0
ω[0] = 1
# ω_dot[0] = -1

θ[1] = 1
ω[1] = 1
# ω_dot[1] = -1

drag_coeff_sim = 0.01

# omega_dot_dot_held = 0

def run():
    ω_dot_commanded = -1

    for i in range(2, N-2):

        ω[i-1] = ω[i-2] + ω_dot_commanded * DT / SIM_RATIO
        # Subtract, due to how this sim is set up re delta theta.
        θ[i] = θ[i-1] - ω[i-1] * DT / SIM_RATIO    

        if i % SIM_RATIO == 0:

            ω_dot_meas = ω[i-1] - ω[i-2]

            EPS = 0.00001

            if abs(ω_dot_meas) < EPS:
                t = -(3 * θ[i]) / (2. * ω[i])
            else:
                inner = 4. * ω[i]**2 - 6. * ω_dot_meas * θ[i]

                if abs(inner) < EPS:
                    
                    pass
                    # todo...
                else:
                    t_a = -(np.sqrt(inner) + 2. * ω[i]) / ω_dot_meas
                    t_b = (np.sqrt(inner) - 2. * ω[i]) / ω_dot_meas
                    
                    if t_a < 0:
                        t = t_b
                        
                    else:
                        t = t_a
            
            ω_dot_dot = 6. * (2. * θ[i] + t * ω[i]) / t**3

            ω_dot_commanded = ω_dot_meas + ω_dot_dot

        # todo: More sophisticated integration method.
        # ω[i] = ω[i-1] + ω_dot[i-1] * DT / SIM_RATIO
        
        # todo: Incorporate drag in sim and control code
     

    plt.plot(θ)
    plt.show()
    plt.plot(ω)
    plt.show()  
    plt.plot(ω_dot)
    plt.show()


def find_time_to_correct(
    θ_0: float,
    ω_0: float,
    ω_dot_0: float,
    θ_tgt: float,
    ω_tgt: float,
) -> float:
    """See Rust code for a description."""

    EPS = 0.000001

    if abs(ω_dot_0) < EPS and abs(θ_0 - θ_tgt) > EPS and abs(ω_tgt + 2. * ω_0) > EPS:
        t = 3. * (θ_tgt - θ_0) / (ω_tgt + 2. * ω_0)
        print("\n\nCondition A")

    elif abs(ω_dot_0) < EPS:
        print("\n\nDivide by 0 on ω_dot_0; failure.")
        t = 6.9
    else:
        inner = 6.*ω_dot_0 * (θ_tgt - θ_0) + (ω_tgt + 2. * ω_0)**2

        if inner < 0.:
            print("No solution exists (imaginary time)")
            t = 6.9
        else:

            t_a = -(np.sqrt(inner) + 2. * ω_0 + ω_tgt) / ω_dot_0
            t_b = (np.sqrt(inner) - 2. * ω_0 - ω_tgt) / ω_dot_0

            print(f"TA: {t_a}, TB: {t_b}")
            
            if t_a > 0. and t_b > 0.:
                t = min(t_a, t_b)
            elif t_a > 0.:
                t = t_a
            elif t_b > 0.:
                t = t_b
            else:
                t = 6.9

        if abs(t - 6.9) < EPS:
            pass
            print("Using our old method?")
            # Set TTC; use our old method

    return t

def plot_analytic():
    θ_tgt = 2.
    ω_tgt = 1

    for θ_0 in [1.5]:
        for ω_0 in [-0.5]:
            ω_dot_0 = 0.336

            # An alternative approach where we specify initial
            # acceleration. This has the advantage of being clearly
            # specified by measured params (theta, omega, omega_dot)

            # todo: You have 2 edge cases, which require deviating from the
            # todo linear correction #1: inner < 0. (no linear jerk solution from current
            # accel) #2. ttc too higgh. We fix both by modifying the acceleration
            # discontinuously.
            # todo: Set a max ttc per theta.

            ttc = find_time_to_correct(θ_0, ω_0, ω_dot_0, θ_tgt, ω_tgt);
                    
            # todo: Which t? a or b may result in negative time; that's a clue

            # j = 6. * (2.* θ_0 + ttc*ω_0) / ttc**3
            j = 6./ ttc**3 * (2.* θ_0 + ttc*ω_0 - 2.*θ_tgt + ttc * ω_tgt) 

            print(f"\n\n ω_dot_0: {ω_dot_0}, j: {j}, ttc: {ttc}")

            n = 30_000

            θ = np.zeros(n)
            ω = np.zeros(n)
            ω_dot = np.zeros(n)

            t = np.linspace(0, 10, n)
            
            # This acceleration integral represents our "delta V" - it's a measure
            # of how much correction energy is needed. Its somewhat analagous to impulse.
            # it may be an analog of ttc.

            # ω_dot_integral = 0
            # ω_dot_integral_over_t = np.zeros(N)

            for i, t_ in enumerate(t):
                θ[i] = θ_0 + ω_0 * t_ + 1/2 * ω_dot_0 * t_**2 + \
                    1/6 * j * t_**3
                
                ω[i] = ω_0 + ω_dot_0 * t_ + 1/2 * j * t_**2 

                ω_dot[i] = ω_dot_0 + j * t_

            # GUess: omega goes with square, theta goes with cube?

            plt.plot(t, θ)
            plt.plot(t, ω)
            plt.plot(t, ω_dot)
    plt.show()

plot_analytic()
# run()



# some data (theta0, omega0 = 1, 1)
# t=1: theta = 0.625, omega = -1.75
