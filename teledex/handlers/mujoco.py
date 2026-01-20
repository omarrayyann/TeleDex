import numpy as np


class MujocoHandler:
    """
    Handler for MuJoCo integration. Add this to a Session to control MuJoCo frames.
    
    Example:
        session = Session()
        mujoco_handler = MujocoHandler(model, data)
        session.add_handler(mujoco_handler)
        
        mujoco_handler.link_body("eef_target")
        session.start()
    """
    
    def __init__(self, model, data):
        """
        Initialize the MuJoCo handler.
        
        Args:
            model: The MuJoCo model object.
            data: The MuJoCo data object.
        """
        # Import mujoco only when needed
        import mujoco
        self.mujoco = mujoco
        
        self.model = model
        self.data = data
        self.linked_frames = []
    
    def update(self, session, latest_data):
        """
        Called by the session on each data update. Updates all linked frames.
        
        Returns:
            bool: True if vibration should be triggered.
        """
        should_vibrate = False
        for linked_frame in self.linked_frames:
            result = linked_frame.update(self.model, self.data, latest_data, self.mujoco)
            if result:
                should_vibrate = True
        return should_vibrate
    
    def link_body(self, name, scale=1.0, position_origin=None, rotation_origin=None, 
                  post_transform=None, pre_transform=None, position_limits=None, 
                  toggle_fn=None, button_fn=None, vibrate_fn=None, 
                  disable_pos=False, disable_rot=False):
        """
        Link a MuJoCo body to be controlled by AR data.

        Args:
            name (str): The name of the body in MuJoCo.
            scale (float): Scalar to multiply positions by.
            position_origin (numpy array): Translation offset.
            rotation_origin (numpy array): Rotation offset matrix (3x3).
            post_transform (numpy array): Post-multiply transform (4x4).
            pre_transform (numpy array): Pre-multiply transform (4x4).
            position_limits (numpy array): Position limits [[xmin,xmax], [ymin,ymax], [zmin,zmax]].
            toggle_fn: Function to call when toggle changes.
            button_fn: Function to call when button is pressed.
            vibrate_fn: Function that returns True when vibration should occur.
            disable_pos (bool): If True, don't update position.
            disable_rot (bool): If True, don't update rotation.
        """
        body_id = self.mujoco.mj_name2id(self.model, self.mujoco.mjtObj.mjOBJ_BODY, name)
        linked_frame = LinkedFrame(
            mujoco_id=body_id,
            frame_type=0,
            scale=scale,
            position_origin=position_origin if position_origin is not None else np.zeros(3),
            rotation_origin=rotation_origin if rotation_origin is not None else np.identity(3),
            post_transform=post_transform if post_transform is not None else np.identity(4),
            pre_transform=pre_transform if pre_transform is not None else np.identity(4),
            position_limits=position_limits,
            toggle_fn=toggle_fn,
            button_fn=button_fn,
            vibrate_fn=vibrate_fn,
            disable_pos=disable_pos,
            disable_rot=disable_rot,
        )
        self.linked_frames.append(linked_frame)
        return linked_frame
            
    def link_site(self, name, scale=1.0, position_origin=None, rotation_origin=None, 
                  post_transform=None, pre_transform=None, position_limits=None, 
                  toggle_fn=None, button_fn=None, vibrate_fn=None, 
                  disable_pos=False, disable_rot=False):
        """
        Link a MuJoCo site to be controlled by AR data.
        """
        site_id = self.mujoco.mj_name2id(self.model, self.mujoco.mjtObj.mjOBJ_SITE, name)
        linked_frame = LinkedFrame(
            mujoco_id=site_id,
            frame_type=1,
            scale=scale,
            position_origin=position_origin if position_origin is not None else np.zeros(3),
            rotation_origin=rotation_origin if rotation_origin is not None else np.identity(3),
            post_transform=post_transform if post_transform is not None else np.identity(4),
            pre_transform=pre_transform if pre_transform is not None else np.identity(4),
            position_limits=position_limits,
            toggle_fn=toggle_fn,
            button_fn=button_fn,
            vibrate_fn=vibrate_fn,
            disable_pos=disable_pos,
            disable_rot=disable_rot,
        )
        self.linked_frames.append(linked_frame)
        return linked_frame

    def link_geom(self, name, scale=1.0, position_origin=None, rotation_origin=None, 
                  post_transform=None, pre_transform=None, position_limits=None, 
                  toggle_fn=None, button_fn=None, vibrate_fn=None, 
                  disable_pos=False, disable_rot=False):
        """
        Link a MuJoCo geom to be controlled by AR data.
        """
        geom_id = self.mujoco.mj_name2id(self.model, self.mujoco.mjtObj.mjOBJ_GEOM, name)
        linked_frame = LinkedFrame(
            mujoco_id=geom_id,
            frame_type=2,
            scale=scale,
            position_origin=position_origin if position_origin is not None else np.zeros(3),
            rotation_origin=rotation_origin if rotation_origin is not None else np.identity(3),
            post_transform=post_transform if post_transform is not None else np.identity(4),
            pre_transform=pre_transform if pre_transform is not None else np.identity(4),
            position_limits=position_limits,
            toggle_fn=toggle_fn,
            button_fn=button_fn,
            vibrate_fn=vibrate_fn,
            disable_pos=disable_pos,
            disable_rot=disable_rot,
        )
        self.linked_frames.append(linked_frame)
        return linked_frame


class LinkedFrame:
    """
    Represents a MuJoCo frame (body, site, or geom) linked to AR tracking data.
    """
    
    def __init__(self, mujoco_id, frame_type, scale, position_origin, rotation_origin, 
                 post_transform, pre_transform, position_limits, toggle_fn, button_fn, 
                 vibrate_fn, disable_pos, disable_rot):
        self.id = mujoco_id
        self.frame_type = frame_type  # 0=body, 1=site, 2=geom
        self.scale = scale
        self.position_origin = position_origin
        self.rotation_origin = rotation_origin
        self.post_transform = post_transform
        self.pre_transform = pre_transform
        self.position_limits = position_limits
        self.button_fn = button_fn
        self.toggle_fn = toggle_fn
        self.vibrate_fn = vibrate_fn
        self.disable_pos = disable_pos
        self.disable_rot = disable_rot
        self.last_toggle = False

    def update(self, mujoco_model, mujoco_data, latest_data, mujoco):
        """
        Update the linked frame based on latest AR data.
        
        Returns:
            bool: True if vibration should be triggered.
        """
        if latest_data["rotation"] is None or latest_data["position"] is None:
            return False
            
        pose = np.identity(4)
        pose[0:3, 0:3] = latest_data["rotation"]
        pose[0:3, 3] = latest_data["position"]

        # Apply scale
        pose[0:3, 3] = pose[0:3, 3] * self.scale
        
        # Apply translation
        if self.position_origin is not None:
            pose[0:3, 3] += self.position_origin
            
        # Apply rotation
        pose[0:3, 0:3] = pose[0:3, 0:3] @ self.rotation_origin

        # Apply transformations
        pose = self.post_transform @ pose @ self.pre_transform

        # Handle toggle
        if latest_data["toggle"] is not None and self.toggle_fn is not None and self.last_toggle != latest_data["toggle"]:
            self.toggle_fn()
            self.last_toggle = latest_data["toggle"]

        # Handle button
        if latest_data["button"] and self.button_fn is not None:
            self.button_fn()
            
        # Convert rotation to quaternion
        quat = np.zeros(4)
        mujoco.mju_mat2Quat(quat, pose[0:3, 0:3].flatten())

        # Apply position limits
        if self.position_limits is not None:
            pose[0, 3] = np.clip(pose[0, 3], self.position_limits[0][0], self.position_limits[0][1])
            pose[1, 3] = np.clip(pose[1, 3], self.position_limits[1][0], self.position_limits[1][1])
            pose[2, 3] = np.clip(pose[2, 3], self.position_limits[2][0], self.position_limits[2][1])

        # Update the appropriate frame type
        if self.frame_type == 0:  # body
            mocap_id = mujoco_model.body(self.id).mocapid[0]
            if mocap_id != -1:
                if not self.disable_rot:
                    mujoco_data.mocap_quat[mocap_id] = quat
                if not self.disable_pos:
                    mujoco_data.mocap_pos[mocap_id] = pose[0:3, 3].tolist()
            else:
                if not self.disable_rot:
                    mujoco_model.body_quat[self.id] = quat
                if not self.disable_pos:
                    mujoco_model.body_pos[self.id] = pose[0:3, 3].tolist()

        elif self.frame_type == 1:  # site
            if not self.disable_rot:
                mujoco_model.site_quat[self.id] = quat
            if not self.disable_pos:
                mujoco_model.site_pos[self.id] = pose[0:3, 3].tolist()

        elif self.frame_type == 2:  # geom
            if not self.disable_rot:
                mujoco_model.geom_quat[self.id] = quat
            if not self.disable_pos:
                mujoco_model.geom_pos[self.id] = pose[0:3, 3].tolist()

        # Check if should vibrate
        if self.vibrate_fn is not None:
            return self.vibrate_fn()
        return False
