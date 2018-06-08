from car import Status, LeadCar, DecentralizedFollowingCar, CentralizedFollowingCar

# Cars will crash if too far from each other
c_init = Status(40, 10, 0)
lc = LeadCar(c_init, 0.5, 0)
c_init = Status(39, 8, 0)
c1 = CentralizedFollowingCar(c_init, 0.5, 0, lc, 1, lc)
c_init = Status(38, 8, 0)
c2 = CentralizedFollowingCar(c_init, 0.5, 0, c1, 1, lc)
c_init = Status(37, 8, 0)
c3 = DecentralizedFollowingCar(c_init, 0.5, 0, c2, 1)
c_init = Status(36, 8, 0)
c4 = DecentralizedFollowingCar(c_init, 0.5, 0, c3, 1)

def p_status(*statuses):
    gt=statuses[0]
    for i in range(1,len(statuses)):
        print("{:10.4f},\t{:.4f},\t{:.4f}".format(statuses[i].d-gt.d, statuses[i].v-gt.v, statuses[i].a-gt.a))

p_status(lc.ground_truth,lc.system_measurement,lc.system_status)
p_status(c1.ground_truth, c1.system_measurement, c1.system_status)
p_status(c2.ground_truth, c2.system_measurement, c2.system_status)
p_status(c3.ground_truth, c3.system_measurement, c3.system_status)
p_status(c4.ground_truth, c4.system_measurement, c4.system_status)

lc.update()
c1.update()
c2.update()
c3.update()
c4.update()

