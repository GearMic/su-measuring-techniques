import numpy as np

Utot = 25.15 # in uV
UtotErr = 0.5
Uscope = 3.35
UscopeErr = 0.2
deltaf = 5000

Uopa = np.sqrt(Utot**2 - Uscope**2)
UopaErr = np.sqrt( (Utot*UtotErr/Uopa)**2 + (Uscope*UscopeErr/Uopa)**2 )
print("Uopa", Uopa, UopaErr)

u = Uopa/np.sqrt(deltaf)*1e3 # in nV/sqrt(Hz)
uErr = UopaErr/np.sqrt(deltaf)*1e3
print("u", u, uErr) 