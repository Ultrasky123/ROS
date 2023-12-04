
import cv2

def objek_terdeteksi ():
    # Load trained cascade classifier
    face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

    # Start webcam
    cap = cv2.VideoCapture(0)

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()

        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect faces in the image
        faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5)
        print (len(faces))

        # Draw a rectangle around the faces and calculate coordinates
        for (x, y, w, h) in faces:
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            x_coord = x + w/2
            y_coord = y + h/2
            print("Object coordinates: ({}, {})".format(x_coord, y_coord))
        

        if len(faces)>0:
            print (x_coord,y_coord)
            return [x_coord,y_coord]
        else :
            print(None)
            return None
    

        # Display the resulting frame
        cv2.imshow('frame', frame)

        # Quit program when 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release webcam and close window
    cap.release()
    cv2.destroyAllWindows()

    