import sympy


def compute_piston_kinematics():
#f = distance from main link extremity to piston attachment (opposite to load cell attachement)
#l1 = distance from main link extremity to piston base
#l2 = distance from piston base to joint rotation axis
#beta angle between c and l1
#alpha = angle between c and d

    a, b, c, d, f, l1, beta = sympy.symbols('a b c d f l1 beta')
#    a = 0.0826
#    b = 0.1143
#    c = 0.0818
#    d = 0.0825
#    f = 0.0812
#    l1 = 0.2358
#    beta = 2.11
    alpha = sympy.symbols('alpha')

    e = sympy.functions.sqrt(c**2 + d**2 - 2*c*d*sympy.cos(alpha))
    theta1 = sympy.functions.acos((e**2 + c**2 - d**2)/(2*e*c)) #angle between e and c
    theta2 = sympy.functions.acos((e**2 + b**2 - a**2)/(2*e*b)) #angle between e and b

    ###case alpha < M_PI
    theta = theta1 + theta2
    l_piston1 = sympy.functions.sqrt(f**2 + l1**2 - 2*f*l1 * sympy.cos(beta-theta))
    
    ##case alpha > M_PI
    theta = theta2 - theta1
    l_piston2 = sympy.functions.sqrt(f**2 + l1**2 - 2*f*l1 * sympy.cos(beta-theta))
   

    ###compute the same thing in the case of a triangle configuration (i.e. when the loal cell is on the piston)
    l_piston3 = e
    
    return [l_piston1, l_piston2, l_piston3]

def print_piston_differential():
    
    alpha = sympy.symbols('alpha')
    for x in compute_piston_kinematics():
        #sympy.printing.print_ccode(sympy.diff(x, alpha))
        print sympy.diff(x, alpha)
        print '\n\n'

def test_piston_kinematics():
    
    l = compute_piston_kinematics()
    dl_piston = []
    l_piston = []
    a, b, c, d, f, l1, beta = sympy.symbols('a b c d f l1 beta')
    alpha = sympy.symbols('alpha')
    for x in l:
        #knee
        temp = x.subs({a:0.0826, b:0.1143, c:0.0818, d:0.0825, f:0.0812, l1:0.2358, beta:2.11})
        #hfe
#        temp = x.subs({a:0.0826, b:0.1143, c:0.0512, d:0.0497, f:0.0813, l1:0.272, beta:1.654})
        l_piston.append(temp)
        dl_piston.append(sympy.diff(temp, alpha))
        
    print l_piston[1].subs({alpha:4.76})
    print l_piston[0].subs({alpha:2.666})
    
    return dl_piston