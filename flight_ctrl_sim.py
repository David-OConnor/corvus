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
θ_target = 10.

θ = np.zeros(N)
ω = np.zeros(N)
ω_dot = np.zeros(N)

θ[0] = 1
ω[0] = 1
ω_dot[0] = -1

drag_coeff_sim = 0.01

# temp: fixing jerk
omega_dot_dot_held = 0

def run():
    ω_dot_current = 0

    for i in range(1, N-2):
        ω_dot[i] = ω[i] - ω[i-1]

        EPS = 0.00001

        t = 0
        if abs(ω_dot_0) < EPS:
            t = -(3 * θ[i]) / (2. * ω[i])
        else:
            inner = 4. * ω[i]**2 - 6. * ω_dot_0 * θ[i]

            if abs(inner) < EPS:
                pass
                # todo...
            else:
                t_a = -(np.sqrt(inner) + 2. * ω[i]) / ω_dot_0
                t_b = (np.sqrt(inner) - 2. * ω[i]) / ω_dot_0
                
                if t_a < 0:
                    t = t_b
                    
                else:
                    t = t_a

        # todo: More sophisticated integration method.
        # ω[i] = ω[i-1] + ω_dot[i-1] * DT / SIM_RATIO
        
        # todo: Incorporate drag in sim and control code

        ω[i] = ω[i-1] + ω_dot_current * DT / SIM_RATIO
        θ[i] = θ[i-1] + ω[i-1] * DT / SIM_RATIO          

    plt.plot(θ)
    plt.show()
    plt.plot(ω)
    plt.show()  
    plt.plot(ω_dot)
    plt.show()


def plot_analytic():
    # t = 1.
    t = 1

    # for θ_0 in [0.5, 1.]:
        # for ω_0 in [0., 1., 2.]:
    for θ_0 in [1]:
        for ω_0 in [0]:
            # time to correct
            # t = θ_0 + ω_0

            # Alternatively, specify initial angular accel
            ω_dot_0 = -1.

            EPS = 0.00001
           

            # Our initial approach of specifying ttc:
            # ω_dot_0 = -(6.*θ_0 + 4.*t*ω_0) / t**2
            
            # todo: Positive values are breaking. Find out why
            # todo: Way to identify excessive TTC, and/or excessive J.

            # todo: 

            # An alternative approach where we specify initial
            # acceleration. This has the advantage of being clearly
            # specified by measured params (theta, omega, omega_dot)
            t = 0
            if abs(ω_dot_0) < EPS:
                t = -(3 * θ_0) / (2. * ω_0)
                print("c")
            else:
                inner = 4. * ω_0**2 - 6. * ω_dot_0 * θ_0
                print(inner, "INNER")
                t_a = (np.sqrt(inner) + 2. * ω_0) / -ω_dot_0
                t_b = (np.sqrt(inner) - 2. * ω_0) / ω_dot_0
                
                if t_a < 0:
                    t = t_b
                    print("b")
                    
                else:
                    t = t_a
                    print("a")

              # todo: Which t? a or b may result in negative time; that's a clue


            ω_dot_dot = 6. * (2.*θ_0 + t*ω_0) / t**3

            print(f"\n\n ω_dot_0: {ω_dot_0}, ω_dot_dot: {ω_dot_dot}, ttc: {t}")

            n = 30_000
            dt = 1e-4

            θ = np.zeros(n)
            ω = np.zeros(n)
            j = np.zeros(n)
            

            # t1_2 = t/2.
            # print(f"\nt=1/2, θ_0={θ_0} ω_0={ω_0}")
            # print("θ",
            # θ_0 + ω_0 * t1_2 + 1/2 * ω_dot_0 * t1_2**2 + \
            #         1/6 * ω_dot_dot * t1_2**3
            # )
            # print("ω",
            #     ω_0 + ω_dot_0 * t1_2 + 1/2 * ω_dot_dot * t1_2**2 
            # )

            # This acceleration integral represents our "delta V" - it's a measure
            # of how much correction energy is needed. Its somewhat analagous to impulse.
            # it may be an analog of ttc.
            ω_dot_integral = 0
            ω_dot_integral_over_t = np.zeros(N)

            for i in range(n):
                t_ = i * dt

                θ[i] = θ_0 + ω_0 * t_ + 1/2 * ω_dot_0 * t_**2 + \
                    1/6 * ω_dot_dot * t_**3
                
                ω[i] = ω_0 + ω_dot_0 * t_ + 1/2 * ω_dot_dot * t_**2 

                ω_dot[i] = ω_dot_0 + ω_dot_dot * t_

                ω_dot_integral += abs(ω_dot[i])

                # ω_dot_integral_over_t[i] = ω_dot_integral

            # print(f"ω_dot integral, {ω_dot_integral/10_000}")
            # plt.plot(ω_dot)
            # plt.plot(ω_dot_integral_over_t)
            # plt.plot(ω_dot)
            # plt.show()

            # n_ = 100
            # ttc = np.zeros(n_)
            # θ_ = np.zeros(n_)

            # for k in range(n):
            #     pass

            # What's the relationsip between ω_0, dθ, ttc, and ω_dot integral?
            # It's likely you need to work back from your main kinematics equation,
            # likely with const jerk.
 
            

            for j in [1, 1e3, 2e3, 3e3, 4e3, 5e3, 6e3, 7e3, 8e3, 9e3, 10e3, 11e3, 12e3]:
                #    print(f"ttc: {1-j/10_000}, θ: {θ[int(j)]}, ω: {ω[int(j)]}")
                pass

            # GUess: omega goes with square, theta goes with cube?

            plt.plot(θ)
    plt.show()

# plot_analytic()
run()



# some data (theta0, omega0 = 1, 1)
# t=1: theta = 0.625, omega = -1.75
