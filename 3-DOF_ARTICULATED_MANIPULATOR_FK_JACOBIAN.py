import numpy as np
import math
import PySimpleGUI as sg
import pandas as pd

# GUI code
sg.theme('Green')

#excel code
EXCEL_FILE = '3-DOF_ARTICULATED_MANIPULATOR_FK_JACOBIAN.xlsx'
df = pd.read_excel(EXCEL_FILE)

#LAYOUT CODE
layout = [
    [sg.Push(), sg.Text('3-DOF ARTICULATED MANIPULATOR CALCULATOR', font=("Century Gothic", 20)), sg.Push()],
    [sg.Push(), sg.Text('Forward Kinematics and Jacobian Matrix', font=("Century Gothic", 15)), sg.Push()],
    

    [sg.Text('Fill out the following fields: (inputs were only an example) ')],
    [sg.Text('a1 ='), sg.InputText('1', key='a1', size=(10,1)), sg.Text('T1 ='), sg.InputText('90',key='T1', size=(10,1)),
        sg.Push(), sg.Button('Jacobian Matrix (J)', font=("Century Gothic", 10), 
        size=(16,0), button_color=('Black','Yellow')),
        sg.Button('Determinant of J', font=("Century Gothic", 10),
        size=(16,0), button_color=('Black','Yellow')),
        sg.Button('Inverse of J', font=("Century Gothic", 10),
        size=(16,0), button_color=('Black','Yellow')),
        sg.Button('Transpose of J', font=("Century Gothic", 10),
        size=(16,0), button_color=('Black','Yellow')), sg.Push()],


    [sg.Text('a2 ='), sg.InputText('1', key='a2', size=(10,1)), sg.Text('T2 ='), sg.InputText('30',key='T2', size=(10,1)),
        sg.Push(), sg.Button('Inverse Kinematics', font=("Century Gothic", 10),
        size=(35,0), button_color=('Black','Orange')), sg.Push()],
    
    
    [sg.Text('a3 ='), sg.InputText('1', key='a3', size=(10,1)), sg.Text('T3 ='), sg.InputText('60',key='T3', size=(10,1)),
        sg.Push(), sg.Button('Path and Trajectory Planning', font=("Century Gothic", 10),
        size=(35,0), button_color=('Black','LightBlue')), sg.Push()],
    
    [sg.Button('Click this before solving Forward Kinematics', font=("Century Gothic", 10),
        size=(40,0), button_color=('Black','White')),sg.Push()],
    [sg.Button('Solve Forward Kinematics', tooltip = ' Select: Click this before solving Forward Kinematics', font=("Century Gothic", 10)),
     sg.Push(), sg.Push(), sg.Text('3-DOF ARTICULATED MANIPULATOR AND ITS KINEMATIC DIAGRAM', font=("Century Gothic", 10)), sg.Push()],
    
    [sg.Frame('H0_3 Transformation Matrix = ',[[
        sg.Output(size=(60,10))]]),
        sg.Push(), sg.Image('articulatedmanipulator.png'), sg.Push(), sg.Image('3dof.png')],

    [sg.Frame('Position Vector: ',[[
        sg.Text('X ='), sg.InputText(key='X', size=(10,1)),
        sg.Text('Y ='), sg.InputText(key='Y', size=(10,1)),
        sg.Text('Z ='), sg.InputText(key='Z', size=(10,1))]])],

    [sg.Text()],
    [sg.Submit(), sg.Button('Clear Input', font=("Century Gothic", 10)), sg.Exit()]
]
#window code
window = sg.Window('3-DOF ARTICULATED MANIPULATOR - Forward Kinematics', layout, resizable=True)

def clear_input():
    for key in values:
        window[key](' ')
    return None

disable_J=window['Jacobian Matrix (J)']
disable_DetJ=window['Determinant of J']
disable_InJ=window['Inverse of J']
disable_TJ=window['Transpose of J']
disable_IK=window['Inverse Kinematics']
disable_PT=window['Path and Trajectory Planning']

while True:
    event, values = window.read()
    if event == sg.WIN_CLOSED or event == 'Exit':
        break
    if event =='Clear Input':
        clear_input()

    if event == 'Click this before solving Forward Kinematics':
        disable_J.update(disabled=True)
        disable_DetJ.update(disabled=True)
        disable_InJ.update(disabled=True)
        disable_TJ.update(disabled=True)
        disable_IK.update(disabled=True)
        disable_PT.update(disabled=True)

#Forward kinematics code
    if event == 'Solve Forward Kinematics':
        #link lengths in cm
        a1 = float(values['a1'])
        a2 = float(values['a2'])
        a3 = float(values['a3'])

        #joint variable thetas in degrees
        T1 = float(values['T1'])
        T2 = float(values['T2'])
        T3 = float(values['T3'])

        #joint variable thetas in radians
        T1 = (T1/180.0)*np.pi
        T2 = (T2/180.0)*np.pi
        T3 = (T3/180.0)*np.pi

        #if joint variable are ds, don't need to convert

        ## DH Parameter Table - DHPT (ONLY EDIT for every new manipulator)
        #Rows = no. of HTM
        #Columns + no. of parameters
        #Theta, alpha, r, d

        DHPT = [[(T1),(90.0/180.0)*np.pi,0,(a1)],
            [(T2),(0.0/180.0)*np.pi,(a2),0],
            [(T3),(0.0/180.0)*np.pi,(a3),0]]
        
        # np.trigo function (DHPT[ROW][COLUMN])

        i = 0
        H0_1 = [[np.cos(DHPT[i][0]),-np.sin(DHPT[i][0])*np.cos(DHPT[i][1]),  np.sin(DHPT[i][0])*np.sin(DHPT[i][1]), DHPT[i][2]*np.cos(DHPT[i][0])],
                [np.sin(DHPT[i][0]), np.cos(DHPT[i][0])*np.cos(DHPT[i][1]), -np.cos(DHPT[i][0])*np.sin(DHPT[i][1]), DHPT[i][2]*np.sin(DHPT[i][0])],
                [0, np.sin(DHPT[i][1]), np.cos(DHPT[i][1]), DHPT[i][3]],
                [0,0,0,1]]

        i = 1
        H1_2 = [[np.cos(DHPT[i][0]),-np.sin(DHPT[i][0])*np.cos(DHPT[i][1]),  np.sin(DHPT[i][0])*np.sin(DHPT[i][1]), DHPT[i][2]*np.cos(DHPT[i][0])],
                [np.sin(DHPT[i][0]), np.cos(DHPT[i][0])*np.cos(DHPT[i][1]), -np.cos(DHPT[i][0])*np.sin(DHPT[i][1]), DHPT[i][2]*np.sin(DHPT[i][0])],
                [0, np.sin(DHPT[i][1]), np.cos(DHPT[i][1]), DHPT[i][3]],
                [0,0,0,1]]

        i = 2
        H2_3 = [[np.cos(DHPT[i][0]),-np.sin(DHPT[i][0])*np.cos(DHPT[i][1]),  np.sin(DHPT[i][0])*np.sin(DHPT[i][1]), DHPT[i][2]*np.cos(DHPT[i][0])],
                [np.sin(DHPT[i][0]), np.cos(DHPT[i][0])*np.cos(DHPT[i][1]), -np.cos(DHPT[i][0])*np.sin(DHPT[i][1]), DHPT[i][2]*np.sin(DHPT[i][0])],
                [0, np.sin(DHPT[i][1]), np.cos(DHPT[i][1]), DHPT[i][3]],
                [0,0,0,1]]

        #transformation matrices from base to end effector
        #print("H0_1 = ")
        #print(np.matrix(H0_1))
        #print("H1_2 = ")
        #print(np.matrix(H1_2))
        #print("H2_3 = ")
        #print(np.matrix(H2_3))

        #dot product of H0_3 = H0_1*H1_2*H2_3
        H0_2 = np.dot(H0_1, H1_2)
        H0_3 = np.dot(H0_2, H2_3)

        #transformation matrix of the manipulator
        print("H0_3 =")
        print(np.matrix(H0_3))

        #position vector x y z
        X0_3 = H0_3[0,3]
        print("X =")
        print(X0_3)

        Y0_3 = H0_3[1,3]
        print("Y =")
        print(Y0_3)

        Z0_3 = H0_3[2,3]
        print("Z =")
        print(Z0_3)

        disable_J.update(disabled=False)
        disable_IK.update(disabled=False)
        disable_PT.update(disabled=False)

    if event == 'Submit':
        df = df.append(values, ignore_index=True)
        df.to_excel(EXCEL_FILE, index=False)
        sg.popup('Data saved!', font=("Century Gothic", 12))
        #clear_input()

    if event == 'Jacobian Matrix (J)':
        Z_1=[[0],[0],[1]]

        try:
            H0_1=np.matrix(H0_1)
        except:
            H0_1= -1 #(NOT A NUMBER)
            sg.popup('WARNING!', font=("Century Gothic", 15))
            sg.popup('Please restart the program and make sure to select "Click this before solving Forward Kinematics"', font=("Century Gothic", 10))
            break

        #R 1-3, C1
        #J1=[[1,0,0],[0,1,0],[0,0,1]]  #FOR PRISMATIC TO!
        #J1=np.dot(J1,Z_1)
        #J1=np.matrix(J1)
        
        J1a=[[1,0,0],[0,1,0],[0,0,1]]
        J1a=np.dot(J1a,Z_1)   #r00 (001)

        J1b_1=H0_3[0:3,3:]
        J1b_1=np.matrix(J1b_1)

        J1b_2=[[0],[0],[0]]
        J1b_2=np.matrix(J1b_2)

        J1b= J1b_1 - J1b_2    #d03-d00

        J1=[[(J1a[1,0]*J1b[2,0])-(J1a[2,0]*J1b[1,0])],
            [(J1a[2,0]*J1b[0,0])-(J1a[0,0]*J1b[2,0])],
            [(J1a[0,0]*J1b[1,0])-(J1a[1,0]*J1b[0,0])]]

        #R1-3, C2
        J2a=H0_1[0:3,0:3]
        J2a=np.dot(J2a,Z_1)  #r01 (001)

        J2b_1=H0_3[0:3,3:]
        J2b_1=np.matrix(J2b_1)

        J2b_2=H0_1[0:3,3:]
        J2b_2=np.matrix(J2b_2)

        J2b= J2b_1 - J2b_2     #d03 -d01

        J2=[[(J2a[1,0]*J2b[2,0])-(J2a[2,0]*J2b[1,0])],
            [(J2a[2,0]*J2b[0,0])-(J2a[0,0]*J2b[2,0])],
            [(J2a[0,0]*J2b[1,0])-(J2a[1,0]*J2b[0,0])]]

        #R1-3, C3
        J3a=H0_2[0:3,0:3] 
        J3a=np.dot(J3a,Z_1)    #r02 (001)    r02=r01*r12

        J3b_1=H0_3[0:3,3:]
        J3b_1=np.matrix(J3b_1)

        J3b_2=H0_2[0:3,3:]
        J3b_2=np.matrix(J3b_2)

        J3b= J3b_1 - J3b_2     #d03 - d02

        J3=[[(J3a[1,0]*J3b[2,0])-(J3a[2,0]*J3b[1,0])],
            [(J3a[2,0]*J3b[0,0])-(J3a[0,0]*J3b[2,0])],
            [(J3a[0,0]*J3b[1,0])-(J3a[1,0]*J3b[0,0])]]

        #J4=[[0],[0],[0]]   #FOR PRISMATIC TO!
        #J4=np.matrix(J4)

        J4=[[1,0,0],[0,1,0],[0,0,1]]
        J4=np.dot(J4,Z_1)   #r00
        J4=np.matrix(J4)

        J5=H0_1[0:3,0:3]
        J5=np.dot(J5,Z_1)    #r01
        J5=np.matrix(J5)

        J6=H0_2[0:3,0:3]
        J6=np.dot(J6,Z_1)     #r02
        J6=np.matrix(J6)

        JM1=np.concatenate((J1,J2,J3),1)
        JM2=np.concatenate((J4,J5,J6),1)

        J=np.concatenate((JM1,JM2),0)
        sg.popup("Jacobian Matrix = ", J)

        DJ=np.linalg.det(JM1)
        if DJ==0.0 or DJ==-0.0:
            disable_InJ.update(disabled=True)
            sg.popup('WARNING! Jacobian Matrix is Non-Invertible.', font=("Century Gothic", 12))

        elif DJ != 0.0 or DJ!=-0.0:
             disable_InJ.update(disabled=False)

        #IJ=np.linalg.inv(JM1)
        #TJ=np.transpose(JM1)

        disable_J.update(disabled=True)
        disable_DetJ.update(disabled=False)
        #disable_InJ.update(disabled=False)
        disable_TJ.update(disabled=False)

    if event == 'Determinant of J':
        #let JM1 become 3x3 pos matrix
        try:
            JM1=np.concatenate((J1,J2,J3),1)
        except:
            JM1= -1 #(NOT A NUMBER)
            sg.popup('WARNING!', font=("Century Gothic", 12))
            sg.popup('Please restart the program and make sure to select "Click this before solving Forward Kinematics"', font=("Century Gothic", 10))
            break
        DJ=np.linalg.det(JM1)
        sg.popup('Determinant of J = ', DJ )

        if DJ==0.0 or DJ==-0.0:
            disable_InJ.update(disabled=True)
            sg.popup('WARNING! Jacobian Matrix is Non-Invertible.', font=("Century Gothic", 12))

    if event == 'Inverse of J':
        try:
            JM1=np.concatenate((J1,J2,J3),1)
        except:
            JM1= -1 #(NOT A NUMBER)
            sg.popup('WARNING!', font=("Century Gothic", 12))
            sg.popup('Please restart the program and make sure to select "Click this before solving Forward Kinematics"', font=("Century Gothic", 10))
            break
        IJ=np.linalg.inv(JM1)
        sg.popup('Inverse of J = ', IJ)

    if event == 'Transpose of J':
        try:
            J=np.concatenate((JM1,JM2),0)
        except:
            J= -1 #(NOT A NUMBER)
            sg.popup('WARNING!', font=("Century Gothic", 12))
            sg.popup('Please restart the program and make sure to select "Click this before solving Forward Kinematics"', font=("Century Gothic", 10))
            break
        TJ=np.transpose(JM1)
        sg.popup('Transpose of J = ', TJ)

    if event == 'Inverse Kinematics':
        sg.popup('Program is still not available', font=("Century Gothic", 12))
    
    if event == 'Path and Trajectory Planning':
        sg.popup('Program is still not available', font=("Century Gothic", 12))


window.close()