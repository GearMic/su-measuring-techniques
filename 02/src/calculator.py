import numpy as np

Utot = 25.15 # in uV
UtotErr = 0.5
Uscope = 3.35
UscopeErr = 0.2
deltaf = 5000

Uopa = np.sqrt(Utot**2 - Uscope**2)
UopaErr = np.sqrt( (Utot*UtotErr/Uopa)**2 + (Uscope*UscopeErr/Uopa)**2 )
print("Uopa", Uopa, UopaErr)

Eo = Uopa/np.sqrt(deltaf)*1e3 # in nV/sqrt(Hz)
EoErr = UopaErr/np.sqrt(deltaf)*1e3
print("u", Eo, EoErr) 


fourkt = 1.6e-20*1e9
R_f = 101e3*1e9 / 1e18 # TODO: remove /1e18
G = 1001

Eni = 1/G * np.sqrt(Eo**2 - fourkt*R_f*G)
EniErr = Eo*EoErr/G**2/Eni
print("Eni", Eni, EniErr)