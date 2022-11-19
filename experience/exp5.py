from pylab import *
import serial  # pour installer la bibliothèque : pip install pyserial
import cv2
import numpy as np
import time
import math as m
import matplotlib.pyplot as plt
import imageio

arduino1=serial.Serial('COM6', 9600,timeout=0.1)


cap = cv2.VideoCapture(1)


xr, xb,yr, yb,wr, wb,hr, hb=0,0,0,0,0,0,0,0


fourcc = cv2.VideoWriter_fourcc(*'XVID')

video=cv2.VideoWriter('video.avi', fourcc, 20.0, (1280,720))
vert=cv2.VideoWriter('vert.avi', fourcc, 20.0, (1280,720),isColor=0)


t=time.time()
t0=time.time()

commande=None
ordre=None


Ltraj=[]

x0b,y0b=0,0
x0r,y0r=0,0

def positionVoitureDepart():
    greenPixel=np.where((frame[:,:,0]<0.8*frame[:,:,1]) & (frame[:,:,0]+frame[:,:,1]+frame[:,:,2]>120) & (frame[:,:,2]<0.9*frame[:,:,1]),255,0)



    greenPixelPos=np.zeros((frame.shape[0],frame.shape[1]))+greenPixel
    greenPixelPos=np.uint8(greenPixelPos)

    contoursGreen,hierarchy=cv2.findContours(greenPixelPos,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    if np.shape(contoursGreen)[0]>0:
        x0b,y0b,wb,hb=cv2.boundingRect(contoursGreen[0])


def positionVoiture():
    greenPixel=np.where((frame[:,:,0]<0.8*frame[:,:,1]) & (frame[:,:,0]+frame[:,:,1]+frame[:,:,2]>120) & (frame[:,:,2]<0.9*frame[:,:,1]),255,0)



    greenPixelPos=np.zeros((frame.shape[0],frame.shape[1]))+greenPixel
    greenPixelPos=np.uint8(greenPixelPos)

    contoursGreen,hierarchy=cv2.findContours(greenPixelPos,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    if np.shape(contoursGreen)[0]>0:
        xb,yb,wb,hb=cv2.boundingRect(contoursGreen[0])

def positionObstacle():

    redPixel=np.where((frame[:,:,0]<200) & (frame[:,:,1]<200) & (frame[:,:,2]>220),255,0)

    redPixelPos=np.zeros((frame.shape[0],frame.shape[1]))+redPixel
    redPixelPos=cv2.dilate(redPixelPos,(21,21),iterations=5)
    redPixelPos=np.uint8(redPixelPos)

    contoursRed,hierarchy= cv2.findContours(redPixelPos,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)


    if np.shape(contoursRed)[0]>0:
        xr,yr,wr,hr=cv2.boundingRect(contoursRed[0])



def ecrireDonnees():


    #Position de la voiture
    ligne=";".join([str(xb+wb/2),str(yb+hb/2),str(t)]) + "\n"
    f=open('testCoordonneeBleu.csv', 'a')
    f.write(ligne)
    f.close()

    #Position de l'obstacle
    ligne=";".join([str(xr+wr/2),str(yr+hr/2),str(t)]) + "\n"
    f=open('testCoordonneeRouge.csv', 'a')
    f.write(ligne)
    f.close()





    #Capture des vidéos
    video.write(frame)
    #vert.write(greenPixelPos)
    #rouge.write(redPixelPos)



def trajectoire():


    xdepassement=100



    xobs,yobs,Dx,Dy=x0r,y0r+10,153,60 #pour la plaque avec les leds rouges

    xDepart=xobs-590


    Kg=0.4

    if xb<xDepart:
        phase=0
        return y0b-7,phase

    if xb<xobs:
        phase=1
        gx=y0b-7 - Kg*(xb-xDepart)

    elif xb<xobs+Dx:
        phase=2
        gx=y0b-7 - Kg*(xobs-xDepart)

    else:
        phase=3
        gx=min(y0b-7 - Kg*(xobs-xDepart)+Kg*(xb-xobs)/1.1,y0b-7)


    return gx,phase

def commande():

    #on commande la différence de vitesse de rotation entre les moteurs à gauche et les moteurs à droite
    #ordre en dessous de 30: le véhicule se déplace vers la droite
    y,phase=trajectoire()
    Ltraj.append(y)

    Kp=0.3

    ordre=30 #position centrale: la voiture avance tout droit

    Dy=yb-y
    if abs(Dy)<6:
        Dy=0
    commande=int(Kp*Dy)

    ordre+=commande


    ordre=min(60,ordre)
    ordre=max(1,ordre)

    #Données sur la commandes
    ligne=";".join([str(phase),str(y),str(Dy),str(commande),str(ordre),str(t)]) + "\n"
    f=open('testPhaseOrdre.csv', 'a')
    f.write(ligne)
    f.close()

    ordre=chr(ordre)

    arduino1.write(ordre.encode('ascii'))


    return True



while(1):
    _, frame = cap.read()
    cv2.imshow('frame',frame)


    greenPixel=np.where((frame[:,:,0]<0.8*frame[:,:,1]) & (frame[:,:,0]+frame[:,:,1]+frame[:,:,2]>120) & (frame[:,:,2]<0.9*frame[:,:,1]),255,0)



    greenPixelPos=np.zeros((frame.shape[0],frame.shape[1]))+greenPixel
    greenPixelPos=np.uint8(greenPixelPos)

    contoursGreen,hierarchy=cv2.findContours(greenPixelPos,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    if np.shape(contoursGreen)[0]>0:
        x0b,y0b,wb,hb=cv2.boundingRect(contoursGreen[0])

    positionVoitureDepart()

    redPixel=np.where((frame[:,:,0]<200) & (frame[:,:,1]<200) & (frame[:,:,2]>220),255,0)

    redPixelPos=np.zeros((frame.shape[0],frame.shape[1]))+redPixel
    redPixelPos=cv2.dilate(redPixelPos,(21,21),iterations=5)
    redPixelPos=np.uint8(redPixelPos)

    contoursRed,hierarchy= cv2.findContours(redPixelPos,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)


    if np.shape(contoursRed)[0]>0:
        x0r,y0r,wr,hr=cv2.boundingRect(contoursRed[0])





    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        arduino1.write(chr(100).encode('ascii'))

        t1=time.time()
        print(x0b)
        print(y0b)
        print(x0r)
        print(y0r)
        arduino1.write(chr(30).encode('ascii'))
        break









while(1):

    _, frame = cap.read()

    cv2.imshow('frame',frame)

    if not (frame is None):

        t=time.time()-t1


        greenPixel=np.where((frame[:,:,0]<0.8*frame[:,:,1]) & (frame[:,:,0]+frame[:,:,1]+frame[:,:,2]>120) & (frame[:,:,2]<0.9*frame[:,:,1]),255,0)



        greenPixelPos=np.zeros((frame.shape[0],frame.shape[1]))+greenPixel
        greenPixelPos=np.uint8(greenPixelPos)

        contoursGreen,hierarchy=cv2.findContours(greenPixelPos,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        if np.shape(contoursGreen)[0]>0:
            xb,yb,wb,hb=cv2.boundingRect(contoursGreen[0])

        positionVoiture()

        redPixel=np.where((frame[:,:,0]<200) & (frame[:,:,1]<200) & (frame[:,:,2]>220),255,0)

        redPixelPos=np.zeros((frame.shape[0],frame.shape[1]))+redPixel
        redPixelPos=cv2.dilate(redPixelPos,(21,21),iterations=5)
        redPixelPos=np.uint8(redPixelPos)

        contoursRed,hierarchy= cv2.findContours(redPixelPos,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)


        if np.shape(contoursRed)[0]>0:
            xr,yr,wr,hr=cv2.boundingRect(contoursRed[0])

        commande()



        ecrireDonnees()






    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break


cv2.destroyAllWindows()
video.release()
vert.release()
arduino1.write("4".encode('ascii'))
arduino1.close()

cap.release()


