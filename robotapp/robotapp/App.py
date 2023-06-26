import rclpy
import vlc
from sensor_msgs.msg import Image
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient


import os
import pickle
import time
from datetime import datetime

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QTableWidgetItem, QMessageBox, QDialog, QLineEdit, QDialogButtonBox, QVBoxLayout
from main_ui import Ui_Form
import sys
import cv2
import MoveGoal as MG



class plapla():
    def __init__(self):
        pass
class RosNode(Node):

    def __init__(self):
        super().__init__('Robot_App')
        self.camera_topic = '/camera/image_raw'
        self.odom_topic = '/odom'
        
        self.image_sub = self.create_subscription(Image, self.camera_topic, self.camera_callback, 1)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 1)
        
        self.bridge = CvBridge()
        self.image = None 
        self.pose = None 

        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')


    def camera_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.image = cv_image
        self.show_image(cv_image)

    def odom_callback(self, data):
        self.pose = data.pose.pose

    def get_current_pose(self):
        if (not self.pose):
            return 0
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = self.pose.position.x
        goal_pose.pose.position.y = self.pose.position.y
        goal_pose.pose.position.z = self.pose.position.z
        goal_pose.pose.orientation.x = self.pose.orientation.x
        goal_pose.pose.orientation.y = self.pose.orientation.y
        goal_pose.pose.orientation.z = self.pose.orientation.z
        goal_pose.pose.orientation.w = self.pose.orientation.w
        return goal_pose

    def get_image(self):
        return self.image



    def show_image(self,img):
        cv2.imshow("Image Window", img)
        cv2.waitKey(3)

    def send_goal(self, posestamped):

        goal_pose = NavigateToPose.Goal()
        goal_pose.pose.header.frame_id = 'map'
        goal_pose.pose.pose.position.x = posestamped.pose.position.x
        goal_pose.pose.pose.position.y = posestamped.pose.position.y
        goal_pose.pose.pose.position.z = posestamped.pose.position.z
        goal_pose.pose.pose.orientation.x = posestamped.pose.orientation.x
        goal_pose.pose.pose.orientation.y = posestamped.pose.orientation.y
        goal_pose.pose.pose.orientation.z = posestamped.pose.orientation.z
        goal_pose.pose.pose.orientation.w = posestamped.pose.orientation.w

        self.get_logger().info('waiting for action server')
        self._action_client.wait_for_server()
        self.get_logger().info('action server detected')
        
        self._send_goal_future = self._action_client.send_goal_async(goal_pose)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        



    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()

        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):

        result = future.result().result
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation succeeded! ')
        else:
            self.get_logger().info(
                'Navigation failed with status: {0}'.format(status))

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback




class Window(Ui_Form):
    def __init__(self, node):
        self.now = datetime.now()

        #Qwidget
        self.Form = QtWidgets.QWidget()
        self.setupUi(self.Form)
        # check hide
        self.ch_hide = 0
        # btn hide menue side action
        self.btn_menue_hide.clicked.connect(self.btn_menue_hide_show)
        ###############################################################
        #home
        self.stackedWidget.setCurrentWidget(self.page_home)
        self.style_sheet_change(self.btn_menue_home)
        # btn_home
        self.btn_menue_home.clicked.connect(self.btn_home)
        ################################################################################
        # camera
        self.btn_menue_camera.clicked.connect(self.btn_camera)
        ################################################################################
        # records
        self.btn_menue_records.clicked.connect(self.btn_records)
        ################################################################################

        #initialize actions home page
        self.inti_actions_data_home()
        #############################################################################
        # initialize actions camera page
        self.inti_camera_page_actions_data()
        #############################################################################
        #inti records page
        self.inti_actions_data_records()
        ############################################################################
        self.ros_node = node

######################home######################################
    def inti_actions_data_home(self):
        #dech locations
        self.location_cache=dict()
        self.load_locations()
        # add
        self.btn_add.clicked.connect(self.btn_add_action)
        # go
        self.btn_go.clicked.connect(self.btn_go_action)
        # update
        self.btn_update_home.clicked.connect(self.btn_update_table_home_action)
        #
        self.lb_total_number_table_home.setText("0")
        self.lb_selected_number_table_home_.setText("0")
        # fixed_error_horz_header_tables
        self.table_home.horizontalHeader().setVisible(True)
        #fill table
        self.fill_table_home()
        #  click table
        self.table_home.clicked.connect(self.signal_click_home_table)
        # Connect the cellDoubleClicked signal to the update_cell_value function
        self.table_home.cellDoubleClicked.connect(self.update_cell_value)
        #search
        self.le_search_home.textChanged.connect(self.search_by_name)
        # Connect the selectionChanged signal of the table widget to update the selected rows label
        self.table_home.itemSelectionChanged.connect(self.update_selected_rows_label)


    def save_MG(self,mg):
        ROOT_DIR = os.path.dirname(
            os.path.abspath(__file__)
        )
        path=os.path.join(ROOT_DIR,"MG",str(mg.name))
        print(path)
        with open(path, 'wb') as out_file:
            pickle.dump(mg, out_file)


    def btn_add_action(self):
        try:
            name=self.le_name.text()
            if name or len(name)>0:
                pose=self.ros_node.get_current_pose()
                mg=MG.MoveGoal(name,pose)
                self.save_MG(mg)
                self.btn_update_table_home_action()
                self.msg_inf("New Location Saved")
            else:
                error="EROOR .. Invalid Name"
                self.msg_worning(error)
            #print(ROOT_DIR)
        except Exception as ex:
            error = "Exception\n" + str(ex)
            self.msg_worning(error)

    def btn_go_action(self):
        try:
            selected_items = self.table_home.selectedItems()
            if len(selected_items) == 2:
                name_item, path_item = selected_items
                print(name_item.text(), path_item.text())
                print("going to: ")
                print(self.location_cache[name_item.text()][1])
                self.ros_node.send_goal(self.location_cache[name_item.text()][1])
            else:
                error = "Error\n" + str("more than one row selected")
                self.msg_worning(error)
        except Exception as ex:
            error = "Exception\n" + str(ex)
            self.msg_worning(error)

    def btn_update_table_home_action(self):
        try:
            #
            self.lb_total_number_table_home.setText("0")
            self.lb_selected_number_table_home_.setText("0")
            ##
            self.location_cache.clear()
            self.load_locations()
            self.fill_table_home()
        except Exception as ex:
            error = "Exception\n" + str(ex)
            self.msg_worning(error)


    def load_locations(self):
        try:
            ROOT_DIR = os.path.dirname(
                os.path.abspath(__file__)
            )
            list_files=os.listdir(os.path.join(ROOT_DIR,'MG'))
            #print(list_files)
            for name in list_files:
                path = os.path.join(ROOT_DIR, "MG",name)
                with open(path, 'rb') as in_file:
                    mg = pickle.load(in_file)
                    self.location_cache[name]=[path,mg.pose]
                    #print(name,mg.pose)
        except Exception as ex:
            error = "Exception\n" + str(ex)
            self.msg_worning(error)

    def fill_table_home(self):
        try:
            #  data to fill into the table
            data=[]
            for name in self.location_cache:
                data.append((name,self.location_cache[name][0],self.location_cache[name][1]))
            
            self.table_home.clearContents()
            # Set the number of rows and columns in the table
            self.table_home.setRowCount(len(data))
            self.table_home.setColumnCount(2)
            #
            self.lb_total_number_table_home.setText(str(len(data)))

            # Set the table headers
            headers = ["Name", "Path"]
            self.table_home.setHorizontalHeaderLabels(headers)

            # Fill the table with data
            for row, row_data in enumerate(data):
                for column, item in enumerate(row_data):
                    table_item = QTableWidgetItem(str(item))
                    self.table_home.setItem(row, column, table_item)
        except Exception as ex:
            error = "Exception\n" + str(ex)
            self.msg_worning(error)

    def update_file_name(self,name,new_name):
        ROOT_DIR = os.path.dirname(
            os.path.abspath(__file__)
        )
        new_path = os.path.join(ROOT_DIR, "MG", new_name)
        os.rename(self.location_cache[name][0], new_path)


    def update_cell_value(self, row, column):
        try:
            if column == 0 :  # Only allow editing "Name" and "Path" columns
                item = self.table_home.item(row, column)
                if item is not None:
                    old_value = item.text()

                    # Create an input dialog to get the new value from the user
                    new_value, ok = self.get_new_value_dialog(old_value)
                    if ok:
                        self.update_file_name(old_value,new_value)
                        #item.setText(new_value)
                        self.btn_update_table_home_action()

        except Exception as ex:
            error = "Exception\n" + str(ex)
            self.msg_worning(error)
            #print(error)


    def get_new_value_dialog(self, old_value):
        # Create and configure an input dialog
        dialog = InputDialog(old_value)
        dialog.setWindowTitle("Edit Value")
        dialog.setModal(True)

        # Execute the dialog and return the new value and dialog status
        if dialog.exec_() == QDialog.Accepted:
            new_value = dialog.get_input_value()
            return new_value, True
        else:
            return old_value, False

    def search_by_name(self, name):
        try:
            name = name.lower()
            for row in range(self.table_home.rowCount()):
                name_item = self.table_home.item(row, 0)  # Name column
                if name_item is not None:
                    row_name = name_item.text().lower()
                    if name in row_name:
                        self.table_home.setRowHidden(row, False)
                    else:
                        self.table_home.setRowHidden(row, True)

        except Exception as ex:
            error = "Exception\n" + str(ex)
            self.msg_worning(error)

    def signal_click_home_table(self,r):
        try:
            pass
        except Exception as ex:
            error = "Exception\n" + str(ex)
            self.msg_worning(error)

    def update_selected_rows_label(self):
        try:
            selected_rows = len(set(item.row() for item in self.table_home.selectedItems()))
            self.lb_selected_number_table_home_.setText(str(selected_rows))
        except Exception as ex:
            error = "Exception\n" + str(ex)
            self.msg_worning(error)
##############################################end home############################################################
##################camera
    def inti_camera_page_actions_data(self):
        self.btn_start_stop_camera.clicked.connect(self.toggle_camera)
        self.btn_start_stop_record.clicked.connect(self.start_stop_record)

        self.ch_record_start_stop=False
        self.VW=None
        self.camera = False
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)

    def inti_record(self,name):    
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        #dt_string = self.now.strftime("%d/%m/%Y %H:%M:%S")
        #print(self.now.time())
        output_file = str(name)+'.avi'    
        ROOT_DIR = os.path.dirname(
            os.path.abspath(__file__)    )
        path = os.path.join(ROOT_DIR, "records", output_file)    
        frame_rate = 10.0
        resolution = (640, 480)    
        video_writer = cv2.VideoWriter(path, fourcc, frame_rate, resolution)
        return video_writer 


    def start_stop_record(self):    
        try:
            if self.camera:
                if self.ch_record_start_stop:            
                    self.VW.release()
                    self.VW=None            
                    self.ch_record_start_stop=False
                    self.btn_start_stop_record.setText("START RECORD")        
                else:
                    name = self.le_name_recoed.text()            
                    if name:
                        self.VW=self.inti_record(name)                
                        self.ch_record_start_stop=True
                        self.btn_start_stop_record.setText("STOP RECORD")
                    else:                
                        error= "Error\n Enter Name"
                        self.msg_worning(error)
            else:
                error = "Error\nopen camera"
                self.msg_worning(error)

        except Exception as ex:        
            error = "Exception\n" + str(ex)
            self.msg_worning(error)


    def toggle_camera(self):
        if self.camera :
            self.stop_camera()
        else:
            self.start_camera()


    def start_camera(self):
        try:
            self.camera=True
            self.btn_start_stop_camera.setText("Stop")
            self.msg_inf("Camera started.")

            self.timer.start(30)
        except Exception as ex:
            error = "Exception\n" + str(ex)
            self.msg_worning(error)


    def stop_camera(self):
        try:
            self.timer.stop()
            self.camera = False

            self.btn_start_stop_camera.setText("Start")
            self.msg_inf("Camera stopped.")
            self.lb_camera.clear()

        except Exception as ex:
            error = "Exception\n" + str(ex)
            self.msg_worning(error)


    def update_frame(self):
        frame = self.ros_node.get_image()
        #record frames (BGR)if self.ch_record_start_stop:
        if self.VW :
            self.VW.write(frame)
        #(frame)
        # if not frame == None:
        # Convert the OpenCV BGR image to RGB
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        # Create a QImage from the frame data
        image = QImage(frame.data, frame.shape[1], frame.shape[0], QImage.Format_RGB888)
        # Create a QPixmap from the QImage
        pixmap = QPixmap.fromImage(image)
        # Scale the QPixmap to fit the label size and set it as the label's pixmap
        scaled_pixmap = pixmap.scaled(self.lb_camera.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.lb_camera.setPixmap(scaled_pixmap)


    def closeEvent(self, event):
        self.stop_camera()
        event.accept()
#####################end camera###############################

######################records######################################
    def inti_actions_data_records(self):
        self.records_cache=dict()
        self.load_records()
        # update
        self.btn_update_records.clicked.connect(self.btn_update_table_records_action)
        #
        self.lb_total_number_records.setText("0")
        self.lb_selected_number_table_records.setText("0")
        # fixed_error_horz_header_tables
        self.table_records.horizontalHeader().setVisible(True)
        # fill table
        self.fill_table_records()

        # search
        self.le_search_records.textChanged.connect(self.search_by_name_records)
        # Connect the selectionChanged signal of the table widget to update the selected rows label
        self.table_records.itemSelectionChanged.connect(self.update_selected_rows_label_records)
        # Connect the cellDoubleClicked signal to the update_cell_value function
        self.table_records.cellDoubleClicked.connect(self.open_video)


    def btn_update_table_records_action(self):
        try:
            #
            self.lb_total_number_records.setText("0")
            self.lb_selected_number_table_records.setText("0")
            ##
            self.records_cache.clear()
            self.load_records()
            self.fill_table_records()
        except Exception as ex:
            error = "Exception\n" + str(ex)
            self.msg_worning(error)

    def load_records(self):
        try:
            ROOT_DIR = os.path.dirname(
                os.path.abspath(__file__)
            )
            list_files=os.listdir(os.path.join(ROOT_DIR,'records'))
            #print(list_files)
            for name in list_files:
                path = os.path.join(ROOT_DIR, "records",name)
                #time=name.split('^')[1]
                self.records_cache[name]=[path]
                #print(name,mg.pose)~
        except Exception as ex:
            error = "Exception\n" + str(ex)
            self.msg_worning(error)





    def fill_table_records(self):
        try:
            #  data to fill into the table
            data=[]
            for name in self.records_cache:
                data.append((name,self.records_cache[name][0]))
            
            # Sample data to fill into the table
            #data = [
               # ( "File 1", "8.30","/path/file1"),
               # ( "File 2","9.0", "/path/file2"),
               # ( "File 3", "6.15","/path/file3"),
            #]
            self.table_records.clearContents()
            # Set the number of rows and columns in the table
            self.table_records.setRowCount(len(data))
            self.table_records.setColumnCount(2)
            #
            self.lb_total_number_records.setText(str(len(data)))

            # Set the table headers
            headers = [ "Name", "Path"]
            self.table_records.setHorizontalHeaderLabels(headers)

            # Fill the table with data
            for row, row_data in enumerate(data):
                for column, item in enumerate(row_data):
                    table_item = QTableWidgetItem(str(item))
                    self.table_records.setItem(row, column, table_item)
        except Exception as ex:
            error = "Exception\n" + str(ex)
            self.msg_worning(error)

    def search_by_name_records(self, name):
        try:
            name = name.lower()
            for row in range(self.table_records.rowCount()):
                name_item = self.table_records.item(row, 0)  # Name column
                if name_item is not None:
                    row_name = name_item.text().lower()
                    if name in row_name:
                        self.table_records.setRowHidden(row, False)
                    else:
                        self.table_records.setRowHidden(row, True)

        except Exception as ex:
            error = "Exception\n" + str(ex)
            self.msg_worning(error)

    def update_selected_rows_label_records(self):
        try:
            selected_rows = len(set(item.row() for item in self.table_records.selectedItems()))
            self.lb_selected_number_table_records.setText(str(selected_rows))
        except Exception as ex:
            error = "Exception\n" + str(ex)
            self.msg_worning(error)

    def open_video(self,row, column):
        try:
            if column==1:
                path = self.table_records.item(row, column).text()
                print(path)
                # Create VLC instance
                vlc_instance = vlc.Instance('--no-xlib')
                # Create VLC media player
                player = vlc_instance.media_player_new()
                # Load video file
                media = vlc_instance.media_new(path)
                # Set the media to the player
                player.set_media(media)
                # Play the video
                player.play()
                # Wait until playback finishes
                while player.get_state() != vlc.State.Ended:
                    time.sleep(1) 
 
                    # Check if the player window is closed 
                    if player.get_state() == vlc.State.Error: 
                        break
                # Clean up the player and instance
                player.stop()
                vlc_instance.release()
        except Exception as ex:
            error = "Exception\n" + str(ex)
            self.msg_worning(error)


##############################################end record############################################################

    def msg_inf(self,m):
        msg=QMessageBox(self.Form)
        msg.setIcon(QMessageBox.Information)
        msg.setText(str(m)+"                         ")
        msg.setWindowTitle("Message")
        #msg.setStyleSheet("\n"
        #                  "font: 75 14pt \"Comic Sans MS\";\n"
        #                  )
        msg.show()
        #msg.setStandardButtons(QMessageBox.Ok )

    def msg_worning(self,m):
        msg=QMessageBox(self.Form)
        msg.setIcon(QMessageBox.Warning)
        msg.setText(str(m)+"                         ")
        msg.setWindowTitle("Warning!")
        #msg.setStyleSheet("\n"
        #                  "font: 75 14pt \"Comic Sans MS\";\n"
        #                  )
        msg.show()
        #msg.setStandardButtons(QMessageBox.Ok )




    def style_sheet_change(self,btn_name):
        style_sheet_btn_menue = self.btn_menue_hide.styleSheet()
        self.btn_menue_home.setStyleSheet(style_sheet_btn_menue)
        self.btn_menue_camera.setStyleSheet(style_sheet_btn_menue)
        self.btn_menue_records.setStyleSheet(style_sheet_btn_menue)
        self.btn_settings.setStyleSheet(style_sheet_btn_menue)
        btn_name.setStyleSheet("QPushButton{\n"
                                          "background-color: rgb(40, 44, 52);\n"
                                          "border:2px solid ;\n"
                                          "font: 10pt \"Arial\";\n"
                                          "border-color: rgb(40, 44, 52);\n"
                                          "border-left-color: rgb(255, 0, 255);\n"
                                          "text-align: left;\n"
                                            "padding-left: 16px;\n"
                                            "color: rgb(249, 249, 249);\n"
                                          "}\n"
                                          )

    def prep_hide_menue(self):
        try:
            self.btn_menue_hide.setText(" ")
            self.btn_menue_home.setText(" ")
            self.btn_menue_camera.setText(" ")
            self.btn_menue_records.setText(" ")
            self.btn_settings.setText(" ")
        except Exception as ex:
            error = "Exception\n" + str(ex)
            self.msg_worning(error)

    def prep_show_menue(self):
        try:
            self.btn_menue_hide.setText("  Hide")
            self.btn_menue_home.setText("  Home")
            self.btn_menue_camera.setText("  Camera")
            self.btn_menue_records.setText("  Records")
            self.btn_settings.setText("  Settings")
        except Exception as ex:
            error = "Exception\n" + str(ex)
            self.msg_worning(error)

    def btn_menue_hide_show(self):
        try:
            w_h=48
            w_sh=180
            ch=self.ch_hide
            #print(width)
            if ch==0:
                self.btn_menue_hide.setMinimumWidth(w_h)
                self.btn_menue_home.setMinimumWidth(w_h)
                self.btn_menue_camera.setMinimumWidth(w_h)
                self.btn_menue_records.setMinimumWidth(w_h)
                self.btn_settings.setMinimumWidth(w_h)
                self.frame_btn_side_menue.setMaximumWidth(w_h)
                self.frame_side_menue.setMaximumWidth(w_h)
                self.frame_settings.setMaximumWidth(w_h)
                self.prep_hide_menue()
                self.ch_hide=1
                #self.frame_side_menue.setContentsMargins(4,0,0,0)
            elif ch==1:
                self.btn_menue_hide.setMinimumWidth(w_sh)
                self.btn_menue_home.setMinimumWidth(w_sh)
                self.btn_menue_camera.setMinimumWidth(w_sh)
                self.btn_menue_records.setMinimumWidth(w_sh)
                self.btn_settings.setMinimumWidth(w_sh)
                self.frame_btn_side_menue.setMaximumWidth(w_sh)
                self.frame_side_menue.setMaximumWidth(w_sh)
                self.frame_settings.setMaximumWidth(w_sh)
                self.prep_show_menue()
                self.ch_hide=0
                #self.frame_side_menue.setContentsMargins(0, 0, 0, 0)
        except Exception as ex:
            error = "Exception\n" + str(ex)
            self.msg_worning(error)

    def btn_home(self):
        try:
            #print("k")
            self.stackedWidget.setCurrentWidget(self.page_home)
            self.style_sheet_change(self.btn_menue_home)
        except Exception as ex:
            error = "Exception\n" + str(ex)
            self.msg_worning(error)
        #

    def btn_camera(self):
        try:
            #print("k")
            self.stackedWidget.setCurrentWidget(self.page_camera)
            self.style_sheet_change(self.btn_menue_camera)
        except Exception as ex:
            error = "Exception\n" + str(ex)
            self.msg_worning(error)
        #

    def btn_records(self):
        try:
            #print("k")
            self.stackedWidget.setCurrentWidget(self.page_records)
            self.style_sheet_change(self.btn_menue_records)
        except Exception as ex:
            error = "Exception\n" + str(ex)
            self.msg_worning(error)
        #



class InputDialog(QDialog):
    def __init__(self, initial_value):
        super().__init__()
        self.setWindowTitle("Edit Value")

        self.input_field = QLineEdit(self)
        self.input_field.setText(initial_value)

        button_box = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        button_box.accepted.connect(self.accept)
        button_box.rejected.connect(self.reject)

        layout = QVBoxLayout(self)
        layout.addWidget(self.input_field)
        layout.addWidget(button_box)

    def get_input_value(self):
        return self.input_field.text()


def main(args=None):
    rclpy.init(args=args)
    ros_node = RosNode()
    #qt stuff
    app = QtWidgets.QApplication(sys.argv)
    window = Window(ros_node)
    window.Form.show()
    
    #
    rclpy.spin(ros_node)
    sys.exit(app.exec_())
    ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
##########################################