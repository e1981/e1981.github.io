"use strict"

// register the application module
b4w.register("111_main", function(exports, require) {

// import modules used by the app
var m_app       = require("app");
var m_cfg       = require("config");
var m_data      = require("data");
var m_preloader = require("preloader");
var m_ver       = require("version");

    
 var m_app       = require("app");
var m_data      = require("data");
var m_cam       = require("camera");
var m_ctl       = require("controls");
var m_scs       = require("scenes");
var m_trans     = require("transform");
var m_obj       = require("objects");
var m_gyro      = require("gyroscope");
var m_util      = require("util");
var m_vec3      = require("vec3");
var m_quat      = require("quat");
    
    
var _last_gyro_quat = m_quat.create();

var _quat_tmp = m_quat.create();
var _quat_tmp2 = m_quat.create();
var _vec3_tmp = m_vec3.create();   
    
    
// detect application mode
var DEBUG = (m_ver.type() == "DEBUG");

// automatically detect assets path
var APP_ASSETS_PATH = m_cfg.get_assets_path("222");

/**
 * export the method to initialize the app (called at the bottom of this file)
 */
exports.init = function() {
    m_app.init({
        canvas_container_id: "main_canvas_container",
        callback: init_cb,
        show_fps: DEBUG,
        console_verbose: DEBUG,
        autoresize: true
    });
}

/**
 * callback executed when the app is initialized 
 */
function init_cb(canvas_elem, success) {

    if (!success) {
        console.log("b4w init failure");
        return;
    }

    m_preloader.create_preloader();

    // ignore right-click on the canvas element
    canvas_elem.oncontextmenu = function(e) {
        e.preventDefault();
        e.stopPropagation();
        return false;
    };

    load();
}

/**
 * load the scene data
 */
function load() {
    m_data.load(APP_ASSETS_PATH + "111.json", load_cb, preloader_cb);
}

/**
 * update the app's preloader
 */
function preloader_cb(percentage) {
    m_preloader.update_preloader(percentage);
}

/**
 * callback executed when the scene data is loaded
 */
function load_cb(data_id, success) {

    if (!success) {
        console.log("b4w load failure");
        return;
    }

    m_app.enable_camera_controls();

    // place your code here
    
 
    
    
    var camobj = m_scs.get_object_by_name("Cube");
  
    //var camobj = m_scs.get_active_camera();

    
    
    
    
    function create_rotation_sensors() {

        //m_ctl.register_device_orientation();
        
        
        
        var obj = m_scs.get_object_by_name("Cube");
       // var obj = m_scs.get_active_camera();
        
        
        
        
        var g_sensor = m_ctl.create_gyro_angles_sensor();
        var save_angles = true;

        var rotate_cb = function(obj, id, pulse) {
            if (pulse > 0) {

                var curr_angles = m_ctl.get_sensor_payload(obj, id, 0);
                //if (m_cam.is_eye_camera(obj)) 
                {
                    var alpha = curr_angles[2];
                    var beta  = curr_angles[1];
                    var gamma = curr_angles[0];

                    // 1) По углам alpha, beta, gamma построить кватернион поворота устройств (значение из датчиков).
                    // http://w3c.github.io/deviceorientation/spec-source-orientation.html
                    var quaternion = _quat_tmp;
                    var c1 = Math.cos(alpha / 2);
                    var c2 = Math.cos(beta  / 2);
                    var c3 = Math.cos(gamma / 2);
                    var s1 = Math.sin(alpha / 2);
                    var s2 = Math.sin(beta  / 2);
                    var s3 = Math.sin(gamma / 2);
                    quaternion[0] = c1 * s2 * c3 - s1 * c2 * s3;
                    quaternion[1] = c1 * c2 * s3 + s1 * s2 * c3;
                    quaternion[2] = s1 * c2 * c3 + c1 * s2 * s3;
                    quaternion[3] = c1 * c2 * c3 - s1 * s2 * s3;

                    // 2) Построить кватернион ориентации устройства (взять угол windows.orientation)
                    
                    var orientation = Math.PI * window.orientation / 180;
                    var screen_quat = m_quat.setAxisAngle(m_util.AXIS_Z,
                            -orientation, _quat_tmp2);
                            

                    // 3) Умножить кватернион из 1) на кватернион из 2)
                    quaternion = m_quat.multiply(quaternion, screen_quat, _quat_tmp);

                    // 4) Взять кватернион поворота на угол Math.PI/2 вокруг оси OX (переход в мировую систему координат в движке)
                    var quat = m_quat.setAxisAngle(m_util.AXIS_X, Math.PI / 2,
                            _quat_tmp2);
                    // 5) Умножить кватернион из 3) на кватернион из 4)
                quaternion = m_quat.multiply(quaternion, quat, _quat_tmp);

                    // Дальше устанавливаем поведение, пусть будет работать гироскоп и touch
                    if (save_angles) {
                        m_quat.copy(quaternion, _last_gyro_quat);
                        save_angles = false;
                    } else {
                        var last_gyro_inv_quat = m_quat.invert(_last_gyro_quat, _last_gyro_quat);
                        var cam_quat = m_trans.get_rotation(obj, _quat_tmp2);
                        var clear_cam_quat = m_quat.multiply(cam_quat, last_gyro_inv_quat, _quat_tmp2);
                        var new_cam_quat = m_quat.multiply(clear_cam_quat, quaternion, _quat_tmp2);

                        var up_axis = m_vec3.transformQuat(m_util.AXIS_MZ,
                                new_cam_quat, _vec3_tmp);
                         // 7) Установить вертикальную ось для камеры (функция camera.set_vertical_axis добавлена в версии 16.01 движка).
                        m_cam.set_vertical_axis(obj, up_axis);
                        // 6) Установить для объекта камеры кватернион поворота
                        m_trans.set_rotation_v(obj, new_cam_quat);
                        m_quat.copy(quaternion, _last_gyro_quat);
                    }
                }
            }
        }
        console.log('tress');
        m_ctl.create_sensor_manifold(obj, "ROTATE_GYRO", 
                m_ctl.CT_CONTINUOUS, [g_sensor], null, rotate_cb);
    }


    create_rotation_sensors();

}



});

// import the app module and start the app by calling the init method
b4w.require("111_main").init();





