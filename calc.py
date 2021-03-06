import numpy as np
from scipy.special import erf


sigma = 1.91/(2.*np.sqrt(np.log(2.)))
delta = 1.91/2.
w = 0.86

c0 = -2./(np.sqrt(np.pi)*sigma)
c1 = (np.exp(-np.power((w+2.*delta)/(2.*sigma), 2)) -
      np.exp(-np.power((w-2.*delta)/(2.*sigma), 2)))
c2 = erf((w+2.*delta)/(2.*sigma))+erf((w-2.*delta)/(2.*sigma))

c = c0*c1/c2

x1 = -14.786
y1 = -26.88
x2 = -20.864
y2 = 26.88
x3 = 16.786
y3 = 26.88
x4 = 22.846
y4 = -26.88
denom = (
    (x4**2+x3**2+x2**2+x1**2) *
    (4*(y4**2+y3**2+y2**2+y1**2)-(y4+y3+y2+y1)**2) +
    (x4+x3+x2+x1)*((y4+y3+y2+y1)*(x4*y4+x3*y3+x2*y2+x1*y1) -
                   (x4+x3+x2+x1)*(y4**2+y3**2+y2**2+y1**2)) +
    (x4*y4+x3*y3+x2*y2+x1*y1)*((x4+x3+x2+x1)*(y4+y3+y2+y1) -
                               4*(x4*y4+x3*y3+x2*y2+x1*y1)))

(dx4*x4+dx3*x3+dx2*x2+dx1*x1)*(4*(y4^2+y3^2+y2^2+y1^2)-(y4+y3+y2+y1)^2)
 +(dx4+dx3+dx2+dx1)*((y4+y3+y2+y1)*(x4*y4+x3*y3+x2*y2+x1*y1)
                    -(x4+x3+x2+x1)*(y4^2+y3^2+y2^2+y1^2))
 +(dx4*y4+dx3*y3+dx2*y2+dx1*y1)*((x4+x3+x2+x1)*(y4+y3+y2+y1)
                                -4*(x4*y4+x3*y3+x2*y2+x1*y1))


thnum = (
    (dy4*x4+dy3*x3+dy2*x2+dy1*x1)*(4*(y4**2+y3**2+y2**2+y1**2) -
                                   (y4+y3+y2+y1)**2) +
    (dy4+dy3+dy2+dy1)*((y4+y3+y2+y1)*(x4*y4+x3*y3+x2*y2+x1*y1) -
                       (x4+x3+x2+x1)*(y4**2+y3**2+y2**2+y1**2)) +
    (dy4*y4+dy3*y3+dy2*y2+dy1*y1)*((x4+x3+x2+x1)*(y4+y3+y2+y1)
                                   -4*(x4*y4+x3*y3+x2*y2+x1*y1)))

th = (
    (dy4*x4+dy3*x3+dy2*x2+dy1*x1)*0.0007057935249234418 -
    (dy4+dy3+dy2+dy1)*0.0007026174540612872 +
    (dy4*y4+dy3*y3+dy2*y2+dy1*y1)*7.967747214956038e-05)

txnum = (
    (dx4+dx3+dx2+dx1)*(
        (x4**2+x3**2+x2**2+x1**2)*(y4**2+y3**2+y2**2+y1**2) -
        (x4*y4+x3*y3+x2*y2+x1*y1)**2) +
    (dx4*x4+dx3*x3+dx2*x2+dx1*x1)*(
        (y4+y3+y2+y1)*(x4*y4+x3*y3+x2*y2+x1*y1)-(x4+x3+x2+x1) *
        (y4**2+y3**2+y2**2+y1**2)) +
    (dx4*y4+dx3*y3+dx2*y2+dx1*y1)*(
        (x4+x3+x2+x1)*(x4*y4+x3*y3+x2*y2+x1*y1) -
        (x4**2+x3**2+x2**2+x1**2)*(y4+y3+y2+y1)))

tx = (
    (dx4+dx3+dx2+dx1)*0.25069945567551805 -
    (dx4*x4+dx3*x3+dx2*x2+dx1*x1)*0.0007026174540612872 -
    (dx4*y4+dx3*y3+dx2*y2+dx1*y1)*7.931892352488745e-05)

tynum = (
    (dy4+dy3+dy2+dy1)*(
        (x4**2+x3**2+x2**2+x1**2)*(y4**2+y3**2+y2**2+y1**2) -
        (x4*y4+x3*y3+x2*y2+x1*y1)**2) +
    (dy4*x4+dy3*x3+dy2*x2+dy1*x1)*(
        (y4+y3+y2+y1)*(x4*y4+x3*y3+x2*y2+x1*y1) -
        (x4+x3+x2+x1)*(y4**2+y3**2+y2**2+y1**2)) +
    (dy4*y4+dy3*y3+dy2*y2+dy1*y1)*(
        (x4+x3+x2+x1)*(x4*y4+x3*y3+x2*y2+x1*y1) -
        (x4**2+x3**2+x2**2+x1**2)*(y4+y3+y2+y1)))


ty = (
    (dy4+dy3+dy2+dy1)*0.25069945567551805 -
    (dy4*x4+dy3*x3+dy2*x2+dy1*x1)*0.0007026174540612872 -
    (dy4*y4+dy3*y3+dy2*y2+dy1*y1)*7.931892352488745e-05)


((x4+x3+x2+x1)*(y4+y3+y2+y1)-4*(x4*y4+x3*y3+x2*y2+x1*y1))/denom
