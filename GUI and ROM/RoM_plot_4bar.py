import numpy as np
import matplotlib.pyplot as plt
import math


len1 = 140  # Length of the fixed link 
ext = 25 # length of extension of foot
len2 = len1 #second foot length
a1=28.75 #min angle old 
a2=57.3 #max angle old
a1new = 30.42 #new min angle
a2new = 80.99 #new max angle
extnew = 83 # new extension 
len1new = 140 #new leg length
len2new = 140 #new leg length
run_max_step = False #reduce data points if this is true as its v ineffcient :( 
dataPoints = 200 #150 gives nice plots

# Define maximum and minimum angles for phi1 and phi2
phi1_min = np.radians(a1)  # Minimum angle for phi1 20 - 20
phi1_max = np.radians(a2)  # Maximum angle for phi1 70 - 70
phi2_min = np.radians(180-a2)  # Minimum angle for phi2 70 -(180-70)
phi2_max = np.radians(180-a1)  # Maximum angle for phi2 20 - (180-20)
#BOTH RANGES OF PHI ARE DEFINED FROM SAME 0 AXIS

#New Leg phis
phi1new_min = np.radians(a1new)  
phi1new_max = np.radians(a2new)  
phi2new_min = np.radians(180-a2new)  
phi2new_max = np.radians(180-a1new)

#defining the range of phis
phi1_range = np.linspace(phi1_min ,phi1_max , dataPoints)
phi2_range = np.linspace(phi2_min ,phi2_max , dataPoints)
# Function to solve for the foot and enfd of leg position based on phi1 and phi2

phi1new_range = np.linspace(phi1new_min ,phi1new_max , dataPoints)
phi2new_range = np.linspace(phi2new_min ,phi2new_max , dataPoints)

'''max step calc (super slow) O(n^n)'''
def max_step(x_pos,z_pos):
    # Tolerance value for approximate equality in z-values
    tolerance = 0.5
    
    # Create an empty list to store pairs of x-values that share the same or approximate z-value
    x_pairs = []
    
    # Loop through the z_positions and group the x-values that are close to each other
    for i in range(len(z_pos)):
        for j in range(i + 1, len(z_pos)):
            if abs(z_pos[i] - z_pos[j]) < tolerance:
                x_pairs.append((x_pos[i], x_pos[j]))
    
    # Now we have pairs of x-values with approximately the same z-value
    # Find the maximum distance between these pairs
    max_distance = 0
    for x1, x2 in x_pairs:
        distance = abs(x2 - x1)
        if distance > max_distance:
            max_distance = distance

    print(f"Maximum distance between two x-values with approximately the same z-value: {max_distance}")

def forward_kinematics(phi1, phi2, l1, l2, t):  
    #p1 and p2 are end poionts of first two bars length l1  
    p1 = np.array([l1*math.cos(phi1),l1*math.sin(phi1)])
    p2 = np.array([l1*math.cos(phi2),l1*math.sin(phi2)])
    
    #Normalised gapo bwetween p1 and p2
    d = np.linalg.norm(p2-p1)
    
    #checks if there is a valid solution
    if(d>2*l2 or d < 1e-9):
        return None, None, None,None,None,None,None,None  
    # Midpoint between p1 and p2
    mid = (p1 + p2) / 2.0
    
    # Half distance between p1 and p2
    h = d / 2.0
  
    # Height from midpoint to intersection points
    offset = math.sqrt(l2**2 - h**2)
  
    # Direction perpendicular to p1 
    dir_perp = np.array([-(p2[1]-p1[1]), p2[0]-p1[0]]) / d
  
    # Two possible intersection points
    C = mid + offset * dir_perp
    D = mid - offset * dir_perp
    
    #normliasation
    p1D_vec = D- p2
    p1C_vec = C - p2
    p1D_unitvec = p1D_vec / np.linalg.norm(p1D_vec)
    p1C_unitvec = p1C_vec / np.linalg.norm(p1C_vec)

    if(-1e-9<D[0]<1e-9 and -1e-9<D[1]<1e-9):
        return None, None, None,None,None, None,None,None
    
    #Two solutions with addiotnal extension on p1
    E1 = D + t * p1D_unitvec
    #this second soultion also does no exist due to geometric constraint of the foot that block inflection
    E2 =(D + t * p1C_unitvec)*0 # remove the 0 if you want to see inflection sol
        
    #Returns all possible solutions for the ROM without foot attachment(C,D) and then with (E)
    return C[0],C[1],D[0],D[1],E1[0],E1[1],E2[0],E2[1]
def main():
    # Initialise lists to store the range of positions for the foot 
    x_positions = np.array([])
    z_positions = np.array([])
    x1_positions = np.array([])
    z1_positions = np.array([])
    #new arrays for new leg designs
    xnew_positions = np.array([])
    znew_positions = np.array([])
    x1new_positions = np.array([])
    z1new_positions = np.array([])
    
    # Iterate through the range of theta and compute the position of the foot
    for ophi1 in phi1_range:
        for ophi2 in phi2_range:
                # Calculate the foot position using forward kinematics
            
            x1, z1,x2,z2,e1,e2,e3,e4 = forward_kinematics(ophi1, ophi2, len1, len2, ext)
            if x1 is None:  # unreachable configuration
                continue
            # Store the positions of the foot
            x_positions= np.append(x_positions,x1)
            z_positions= np.append(z_positions,z1)
            x_positions= np.append(x_positions,x2)
            z_positions= np.append(z_positions,z2)
            x1_positions= np.append(x1_positions,e1)
            z1_positions= np.append(z1_positions,e2)
            x1_positions= np.append(x1_positions,e3)
            z1_positions= np.append(z1_positions,e4)
    
        
    # transform of flip in X axis is provided as the maths was done in the page up directions but foot logic is pointed down 
    z_positions = z_positions*-1 
    z1_positions = z1_positions*-1
    
    for phi1new in phi1new_range:
        for phi2new in phi2new_range:
                # Calculate the foot position using forward kinematics
            
            x1new, z1new,x2new,z2new,e1new,e2new,e3new,e4new = forward_kinematics(phi1new, phi2new, len1new, len2new, extnew)
            if x1new is None:  # unreachable configuration
                continue
            # Store the positions of the foot
            xnew_positions= np.append(xnew_positions,x1new)
            znew_positions= np.append(znew_positions,z1new)
            xnew_positions= np.append(xnew_positions,x2new)
            znew_positions= np.append(znew_positions,z2new)
            x1new_positions= np.append(x1new_positions,e1new)
            z1new_positions= np.append(z1new_positions,e2new)
            x1new_positions= np.append(x1new_positions,e3new)
            z1new_positions= np.append(z1new_positions,e4new)
    
    znew_positions = znew_positions*-1 
    z1new_positions = z1new_positions*-1
    if run_max_step == True:
        max_step(x1_positions,z1_positions)
        #max_step(x1new_positions,z1new_positions)
    
    # Plot the range of motion for the end positions
    '''REMEBER TO FIX SCALING OF AXIS'''
    indice1 = [i for i, x in enumerate(x_positions) if -25.05<x < -24.95]
    plt.figure(figsize=(8, 8))
    plt.scatter(x_positions, z_positions, color='#FFE45E', label="Working Area Point D", s=.02)
    plt.scatter(x1_positions, z1_positions, color='#6387CB', label="Working Area Point E", s=.02)
    #plt.axhline(y= z_positions[indice1[-1]], color='r', linestyle='--')
    #plt.axhline(y= z_positions[indice1[0]], color='r', linestyle='--')
    plt.title('Range of Motion for the End (4-Bar Leg %.2f - %.2f deg)'% (a1,a2)) 
    plt.xlabel('X Position [mm]')
    plt.ylabel('Z Position [mm]')
    #plt.xlim(-70,70) 
    #plt.ylim((-300, 0)) 
    plt.grid(True)
    plt.legend(markerscale = 50)
    plt.show()
    
    indice2 = [i for i, x in enumerate(xnew_positions) if -25.05<x < -24.95]
    plt.figure(figsize=(8, 8))
    plt.scatter(xnew_positions, znew_positions, color='#FFE45E', label="Working Area Point D", s=.02)
    plt.scatter(x1new_positions, z1new_positions, color='#6387CB', label="Working Area Point E", s=.02)
    #plt.axhline(y= znew_positions[indice2[-1]], color='r', linestyle='--')
    #plt.axhline(y= znew_positions[indice2[0]], color='r', linestyle='--')
    plt.title('Range of Motion for New Design (4-Bar Leg %.2f - %.2f deg)'% (a1new,a2new)) 
    plt.xlabel('X Position [mm]')
    plt.ylabel('Z Position [mm]')
    plt.grid(True)
    plt.legend(markerscale = 50)
    plt.show()
    
    plt.figure(figsize=(8, 8))
    plt.scatter(x_positions, z_positions, color='#FFE45E', label="Orignal Working Area", s=.02)
    plt.scatter(xnew_positions, znew_positions, color='#6387CB', label="New Working Area", s=.02)
    #plt.axhline(y= z_positions[indice1[-1]], color='green', linestyle='--')
    #plt.axhline(y= z_positions[indice1[0]], color='green', linestyle='--')
    #plt.axhline(y= znew_positions[indice2[-1]], color='magenta', linestyle='--')
    #plt.axhline(y= znew_positions[indice2[0]], color='magenta', linestyle='--')
    plt.title('Range of Motion for Point D of Orignal Vs Old Leg Design') 
    plt.xlabel('X Position [mm]')
    plt.ylabel('Z Position [mm]')
    plt.grid(True)
    
    plt.legend(markerscale = 50)
    plt.show()
    
    plt.figure(figsize=(8, 8))
    plt.scatter(x1_positions, z1_positions, color='#FFE45E', label="Orignal Foot Working Area", s=.02)
    plt.scatter(x1new_positions, z1new_positions, color='#6387CB', label="New Foot Working Area", s=.02)
    plt.title('Range of Motion for Point E of Orignal Vs Old Leg Design') 
    plt.xlabel('X Position [mm]')
    plt.ylabel('Z Position [mm]')
    plt.grid(True)
    plt.legend(markerscale = 50)
    plt.show()
    
    
    print("Change in working range", (abs(znew_positions[indice2[-1]]-znew_positions[indice2[0]]))-abs(z_positions[indice1[-1]]-z_positions[indice1[0]]))
    
main()