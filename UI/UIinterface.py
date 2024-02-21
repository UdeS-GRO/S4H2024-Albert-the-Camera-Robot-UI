# Import Module
from tkinter import *
from PIL import Image, ImageTk

# create root window
root = Tk()

# root window title and dimension
root.title("Welcome to GeekForGeeks")
# Set geometry(widthxheight)
root.geometry('1050x500')

# Load and display an image
image = Image.open("C:\\Users\\ABB\\Documents\\GitHub\\Albert-the-Camera-Robot-UI\\UI\\logo.png")
image = image.resize((100, 50), Image.BICUBIC)
image = ImageTk.PhotoImage(image)

image_label = Label(root, image=image)
image_label.pack(anchor=NW, padx=10, pady=10)

# adding menu bar in root window
# new item in menu bar labelled as 'New'
# adding more items in the menu bar
menu = Menu(root)
item = Menu(menu)
item.add_command(label='New')
menu.add_cascade(label='File', menu=item)
root.config(menu=menu)

# adding a label to the root window
lbl = Label(root, text="Are you a Geek?")
lbl.pack()

# adding Entry Field
txt = Entry(root, width=10)
txt.pack()


# function to display user text when
# button is clicked
def clicked():
    res = "You wrote " + txt.get()
    lbl.configure(text=res)


# button widget with red color text inside
btn = Button(root, text="Click me",
             fg="red", command=clicked)
# Set Button Grid
btn.pack()

# Execute Tkinter
root.mainloop()
