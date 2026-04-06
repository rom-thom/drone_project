


use anyhow::{anyhow, Result};


use crate::types::Px4Client;
use mavlink::common::MavMessage;



use mavlink::common::*;


impl Px4Client {
    pub fn send_gcs_heartbeat(&mut self) -> Result<()> {
        // For a ground station / external controller:
        let hb = HEARTBEAT_DATA {
            custom_mode: 0,
            mavtype: MavType::MAV_TYPE_GCS,
            autopilot: MavAutopilot::MAV_AUTOPILOT_INVALID,
            base_mode: MavModeFlag::empty(),
            system_status: MavState::MAV_STATE_ACTIVE,
            mavlink_version: 3,
        };
        self.send(&MavMessage::HEARTBEAT(hb))
    }

    /// Offboard setpoint: position in LOCAL_NED.
    pub fn send_offboard_setpoint_local_ned(&mut self, x: f32, y: f32, z: f32, yaw_rad: f32) -> Result<()> {
        self.ensure_target()?;

        // Ignore velocities + accelerations + yaw_rate. Keep position + yaw. // TODO This may change in the future
        
        let type_mask =
                PositionTargetTypemask::POSITION_TARGET_TYPEMASK_VX_IGNORE
                | PositionTargetTypemask::POSITION_TARGET_TYPEMASK_VY_IGNORE
                | PositionTargetTypemask::POSITION_TARGET_TYPEMASK_VZ_IGNORE
                | PositionTargetTypemask::POSITION_TARGET_TYPEMASK_AX_IGNORE
                | PositionTargetTypemask::POSITION_TARGET_TYPEMASK_AY_IGNORE
                | PositionTargetTypemask::POSITION_TARGET_TYPEMASK_AZ_IGNORE
                | PositionTargetTypemask::POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;


        let sp = SET_POSITION_TARGET_LOCAL_NED_DATA {
            time_boot_ms: self.time_elapsed_ms(),
            target_system: self.target_sys,
            target_component: self.target_comp,
            coordinate_frame: MavFrame::MAV_FRAME_LOCAL_NED,
            type_mask,
            x,
            y,
            z,
            vx: 0.0,
            vy: 0.0,
            vz: 0.0,
            afx: 0.0,
            afy: 0.0,
            afz: 0.0,
            yaw: yaw_rad,
            yaw_rate: 0.0,
        };

        self.send(&MavMessage::SET_POSITION_TARGET_LOCAL_NED(sp))
    }

    pub fn arm(&mut self) -> Result<()> {
        self.ensure_target()?;
        let cmd = COMMAND_LONG_DATA {
            target_system: self.target_sys,
            target_component: self.target_comp,
            command: MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
            confirmation: 0,
            param1: 1.0, // arm
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            param5: 0.0,
            param6: 0.0,
            param7: 0.0,
        };
        self.send(&MavMessage::COMMAND_LONG(cmd))?;
        let ack = self.wait_command_ack(MavCmd::MAV_CMD_COMPONENT_ARM_DISARM, 2000)?;
        if ack.result == MavResult::MAV_RESULT_ACCEPTED {
            Ok(())
        } else {
            Err(anyhow!("Arm rejected: {:?}", ack.result))
        }
    }

    /// Switch to PX4 Offboard mode via MAV_CMD_DO_SET_MODE.
    ///
    /// PX4 custom main mode OFFBOARD is 6. :contentReference[oaicite:2]{index=2}
    /// A common way is: base_mode enables custom_mode, and custom_mode encodes main/sub modes.
    pub fn set_mode_offboard(&mut self) -> Result<()> {
        self.ensure_target()?;

        let base_mode = MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

        let px4_main_mode_offboard: u32 = 6; // PX4_CUSTOM_MAIN_MODE_OFFBOARD :contentReference[oaicite:3]{index=3}
        let px4_sub_mode: u32 = 0;

        // PX4 encoding: main mode in bits 16..23, sub-mode in 24..31 (common convention in tooling). :contentReference[oaicite:4]{index=4}
        let custom_mode: u32 = (px4_main_mode_offboard << 16) | (px4_sub_mode << 24);

        let cmd = COMMAND_LONG_DATA {
            target_system: self.target_sys,
            target_component: self.target_comp,
            command: MavCmd::MAV_CMD_DO_SET_MODE,
            confirmation: 0,
            param1: base_mode.bits() as f32,
            param2: custom_mode as f32,
            param3: 0.0,
            param4: 0.0,
            param5: 0.0,
            param6: 0.0,
            param7: 0.0,
        };

        self.send(&MavMessage::COMMAND_LONG(cmd))?;
        let ack = self.wait_command_ack(MavCmd::MAV_CMD_DO_SET_MODE, 2000)?;
        if ack.result == MavResult::MAV_RESULT_ACCEPTED {
            Ok(())
        } else {
            Err(anyhow!("Offboard mode rejected: {:?}", ack.result))
        }
    }
}
