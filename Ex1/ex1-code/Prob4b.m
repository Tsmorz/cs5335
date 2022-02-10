syms th1 th2 th3 L1 L2
% create the arm in 4-b.
L1 = 1; L2 = 1;
J1 = Revolute('alpha', 0, 'a', 0, 'd', 0);
J2 = Revolute('alpha', pi/2, 'a', 0, 'd', L1);
J3 = Prismatic('alpha', pi/2, 'a', L2, 'theta', 0, 'qlim', 1);
arm_b = SerialLink([J1 J2 J3], 'name', 'arm_b');
arm_b.teach([0, 0, .5])