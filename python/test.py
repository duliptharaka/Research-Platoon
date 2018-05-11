from car import Status, LeadCar, DecentralizedFollowingCar, CentralizedFollowingCar

# Cars will crash if too far from each other
c_init = Status(40, 10, 0, 0)
lc = LeadCar(c_init, 0, 0)
c_init = Status(39, 8, 0, 0)
c1 = CentralizedFollowingCar(c_init, 0, 0, lc, 1, lc)
c_init = Status(38, 8, 0, 0)
c2 = CentralizedFollowingCar(c_init, 0, 0, c1, 1, lc)
c_init = Status(37, 8, 0, 0)
c3 = DecentralizedFollowingCar(c_init, 0, 0, c2, 1)
c_init = Status(36, 8, 0, 0)
c4 = DecentralizedFollowingCar(c_init, 0, 0, c3, 1)

lc.ground_truth.d, lc.ground_truth.v, lc.ground_truth.a
c1.ground_truth.d, c1.ground_truth.v, c1.ground_truth.a
c2.ground_truth.d, c2.ground_truth.v, c2.ground_truth.a
c3.ground_truth.d, c3.ground_truth.v, c3.ground_truth.a
c4.ground_truth.d, c4.ground_truth.v, c4.ground_truth.a

lc.update()
c1.update()
c2.update()
c3.update()
c4.update()

