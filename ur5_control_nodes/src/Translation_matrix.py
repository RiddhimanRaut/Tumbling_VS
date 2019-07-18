import numpy as np
from math import *

def calc_eff_pos(Q1,Q2,Q3,Q4,Q5,Q6):
    pos = np.array([ 0.047324999984994065016508102416992*sin(Q1 + Q2 + Q3 + Q4) - 0.21249999996507540345191955566406*cos(Q1 + Q2) - 0.020574999991367803886532783508301*sin(Q1 + Q2 + Q3 + Q4 + Q5) + 0.041149999997287523001432418823242*sin(Q1 + Q5) - 0.047324999999545980244874954223633*sin(Q1 - 1.0*Q2 - 1.0*Q3 - 1.0*Q4) - 0.21250000005238689482212066650391*cos(Q1 - 1.0*Q2) + 0.020574999998643761500716209411621*sin(Q1 + Q2 + Q3 + Q4 - 1.0*Q5) + 0.041149999997287523001432418823242*sin(Q1 - 1.0*Q5) - 0.19612499995855614542961120605469*cos(Q1 + Q2 + Q3) + 0.10914999997476115822792053222656*sin(Q1) - 0.020575000009557697921991348266602*sin(Q1 - 1.0*Q2 - 1.0*Q3 - 1.0*Q4 + Q5) + 0.020574999998643761500716209411621*sin(Q1 - 1.0*Q2 - 1.0*Q3 - 1.0*Q4 - 1.0*Q5) - 0.19612500004586763679981231689453*cos(Q1 - 1.0*Q2 - 1.0*Q3), 0.020574999991367803886532783508301*cos(Q1 + Q2 + Q3 + Q4 + Q5) - 0.047324999984994065016508102416992*cos(Q1 + Q2 + Q3 + Q4) - 0.19612500004586763679981231689453*sin(Q1 - 1.0*Q2 - 1.0*Q3) - 0.041149999997287523001432418823242*cos(Q1 + Q5) + 0.047324999999545980244874954223633*cos(Q1 - 1.0*Q2 - 1.0*Q3 - 1.0*Q4) - 0.21249999996507540345191955566406*sin(Q1 + Q2) - 0.020574999998643761500716209411621*cos(Q1 + Q2 + Q3 + Q4 - 1.0*Q5) - 0.041149999997287523001432418823242*cos(Q1 - 1.0*Q5) - 0.21250000005238689482212066650391*sin(Q1 - 1.0*Q2) - 0.10914999997476115822792053222656*cos(Q1) + 0.020575000009557697921991348266602*cos(Q1 - 1.0*Q2 - 1.0*Q3 - 1.0*Q4 + Q5) - 0.19612499995855614542961120605469*sin(Q1 + Q2 + Q3) - 0.020574999998643761500716209411621*cos(Q1 - 1.0*Q2 - 1.0*Q3 - 1.0*Q4 - 1.0*Q5), 0.041149999990011565387248992919922*cos(Q2 + Q3 + Q4 + Q5) - 0.041150000011839438229799270629883*cos(Q2 + Q3 + Q4 - 1.0*Q5) - 0.39224999997531995177268981933594*sin(Q2 + Q3) - 0.000000000016880012165332910351045470065401*cos(Q5) - 0.094649999984540045261383056640625*cos(Q2 + Q3 + Q4) - 0.42499999998835846781730651855469*sin(Q2) + 0.089458999980706721544265747070312]
                 )

    return pos