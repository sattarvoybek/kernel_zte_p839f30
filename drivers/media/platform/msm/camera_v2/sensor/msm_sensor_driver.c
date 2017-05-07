/* Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define SENSOR_DRIVER_I2C "camera"
/* Header file declaration */
#include "msm_sensor.h"
#include "msm_sd.h"
#include "camera.h"
#include "msm_cci.h"
#include "msm_camera_dt_util.h"
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#ifdef CONFIG_SENSOR_INFO 
 extern void msm_sensorinfo_set_rear_sensor_index(uint16_t index);
 extern void msm_sensorinfo_set_front_sensor_index(uint16_t index);
#endif 
/* Logging macro */
#undef CDBG
#define CDBG(fmt, args...) pr_debug(fmt, ##args)

#define SENSOR_MAX_MOUNTANGLE (360)
#define CONFIG_T4K35
static struct v4l2_file_operations msm_sensor_v4l2_subdev_fops;

/* Static declaration */
static struct msm_sensor_ctrl_t *g_sctrl[MAX_CAMERAS];

static bool g_if_back_camera_id_read = false;
static bool g_if_front_camera_id_read = false;
static char g_back_camera_status [128]  = {0};
static char g_front_camera_status [128]  = {0};

//zoupeng imx214 otp start
struct eeprom_block_info 
{	
    int slave_id;	
    uint32_t start_addr;	
    int valid_size;	
    enum msm_camera_i2c_reg_addr_type addr_t;	
    enum msm_camera_i2c_data_type data_t;	
    int delay;
};
//zoupeng imx214 otp end

#if defined(CONFIG_T4K35)
static int32_t t4k35_sensor_i2c_read(struct msm_sensor_ctrl_t *s_ctrl,
	uint16_t addr, uint16_t *data,
	enum msm_camera_i2c_data_type data_type)
{
    int32_t rc = 0;
    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
            s_ctrl->sensor_i2c_client,
            addr,
            data, data_type);
    return rc;
}
static int32_t t4k35_sensor_i2c_write(struct msm_sensor_ctrl_t *s_ctrl,
	uint16_t addr, uint16_t data,
	enum msm_camera_i2c_data_type data_type)
{
    int32_t rc = 0;
    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
        i2c_write(s_ctrl->sensor_i2c_client, addr, data, data_type);
    if (rc < 0) {
        msleep(100);
        rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
               i2c_write(s_ctrl->sensor_i2c_client, addr, data, data_type);
    }
	return rc;
}
#define SET_T4k35_REG(reg, val) t4k35_sensor_i2c_write(s_ctrl,reg,val,MSM_CAMERA_I2C_BYTE_DATA)
#define GET_T4k35_REG(reg, val) t4k35_sensor_i2c_read(s_ctrl,reg,&val,MSM_CAMERA_I2C_BYTE_DATA)
typedef struct t4k35_otp_struct 
{
  uint8_t LSC[53];              /* LSC */
  uint8_t AWB[8];               /* AWB */
  uint8_t Module_Info[9];
  uint16_t AF_Macro;
  uint16_t AF_Inifity;
} st_t4k35_otp;
#define T4k35_OTP_PSEL 0x3502
#define T4k35_OTP_CTRL 0x3500
#define T4k35_OTP_DATA_BEGIN_ADDR 0x3504
#define T4k35_OTP_DATA_END_ADDR 0x3543
static uint16_t t4k35_otp_data[T4k35_OTP_DATA_END_ADDR - T4k35_OTP_DATA_BEGIN_ADDR + 1] = {0x00};
static uint16_t t4k35_otp_data_backup[T4k35_OTP_DATA_END_ADDR - T4k35_OTP_DATA_BEGIN_ADDR + 1] = {0x00};
static uint16_t t4k35_r_golden_value=0x50;
static uint16_t t4k35_g_golden_value=0x90;
static uint16_t t4k35_b_golden_value=0x5D;
static uint16_t t4k35_af_macro_pos=500;
static uint16_t t4k35_af_inifity_pos=100;
static void t4k35_otp_set_page(struct msm_sensor_ctrl_t *s_ctrl, uint16_t page)
{
    SET_T4k35_REG(T4k35_OTP_PSEL, page);
}
static void t4k35_otp_access(struct msm_sensor_ctrl_t *s_ctrl)
{
	uint16_t reg_val;
	GET_T4k35_REG(T4k35_OTP_CTRL, reg_val);
	SET_T4k35_REG(T4k35_OTP_CTRL, reg_val | 0x80);
	usleep(30);
}
static void t4k35_otp_read_enble(struct msm_sensor_ctrl_t *s_ctrl, uint8_t enble)
{
    if (enble)
        SET_T4k35_REG(T4k35_OTP_CTRL, 0x01);
    else
        SET_T4k35_REG(T4k35_OTP_CTRL, 0x00);
}
static int32_t t4k35_otp_read_data(struct msm_sensor_ctrl_t *s_ctrl, uint16_t* otp_data)
{
    uint16_t i = 0;
    for (i = 0; i <= (T4k35_OTP_DATA_END_ADDR - T4k35_OTP_DATA_BEGIN_ADDR); i++)
	{
        GET_T4k35_REG(T4k35_OTP_DATA_BEGIN_ADDR+i,otp_data[i]);
    }
    return 0;
}
static void t4k35_update_awb(struct msm_sensor_ctrl_t *s_ctrl, struct t4k35_otp_struct *p_otp)
{
  uint16_t rg,bg,r_otp,g_otp,b_otp;
  r_otp=p_otp->AWB[1]+(p_otp->AWB[0]<<8);
  g_otp=(p_otp->AWB[3]+(p_otp->AWB[2]<<8)+p_otp->AWB[5]+(p_otp->AWB[4]<<8))/2;
  b_otp=p_otp->AWB[7]+(p_otp->AWB[6]<<8);
  rg = 256*(t4k35_r_golden_value *g_otp)/(r_otp*t4k35_g_golden_value);
  bg = 256*(t4k35_b_golden_value*g_otp)/(b_otp*t4k35_g_golden_value);
  CDBG("r_golden=0x%x,g_golden=0x%x, b_golden=0x%0x\n", t4k35_r_golden_value,t4k35_g_golden_value,t4k35_b_golden_value);
  CDBG("r_otp=0x%x,g_opt=0x%x, b_otp=0x%0x\n", r_otp,g_otp,b_otp);
  CDBG("rg=0x%x, bg=0x%0x\n", rg,bg);
  SET_T4k35_REG(0x0212, rg >> 8);//0x0212[1:0]:DG_GA_RED[9:8]
  SET_T4k35_REG(0x0213, rg & 0xff);//0x0213[7:0]:DG_GA_RED[7:0]
  SET_T4k35_REG(0x0214, bg >> 8);//0x0214[1:0]:DG_GA_BLUE[9:8]
  SET_T4k35_REG(0x0215, bg & 0xff);//0x0215[7:0]:DG_GA_BLUE[7:0]
}
static void t4k35_update_lsc(struct msm_sensor_ctrl_t *s_ctrl, struct t4k35_otp_struct *p_otp)
{
  uint16_t addr;
  int i;
  addr = 0x323A;
  SET_T4k35_REG(addr, p_otp->LSC[0]);
  addr = 0x323E;
  for(i = 1; i < 53; i++) 
  {
    SET_T4k35_REG(addr++, p_otp->LSC[i]);
  }
  SET_T4k35_REG(0x3237,0x80);
}
static int32_t t4k35_otp_init_lsc_awb(struct msm_sensor_ctrl_t *s_ctrl, struct t4k35_otp_struct *p_otp)
{
  int i,j;
  uint16_t check_sum=0x00;
  for(i = 3; i >= 0; i--) 
  {
  	t4k35_otp_read_enble(s_ctrl, 1);
	t4k35_otp_set_page(s_ctrl, i);
    t4k35_otp_access(s_ctrl);
	CDBG("otp lsc data area data: %d \n", i);
    t4k35_otp_read_data(s_ctrl, t4k35_otp_data);
	CDBG("otp lsc backup area data: %d\n", i + 6);
	t4k35_otp_set_page(s_ctrl, i+6);
    t4k35_otp_access(s_ctrl);
	t4k35_otp_read_data(s_ctrl, t4k35_otp_data_backup);
  	t4k35_otp_read_enble(s_ctrl, 0);
    for(j = 0; j < 64; j++) 
	{
        t4k35_otp_data[j]=t4k35_otp_data[j]|t4k35_otp_data_backup[j];
    }
    if (0 == t4k35_otp_data[0]) 
	{
      continue;
    } 
	else 
	{
	  for(j = 2; j < 64; j++) 
	  {
        check_sum=check_sum+t4k35_otp_data[j];
      }
	  if((check_sum & 0xFF) == t4k35_otp_data[1])
	  {
	  	CDBG("otp lsc checksum ok!\n");
		for(j=3;j<=55;j++)
		{
			p_otp->LSC[j-3]=t4k35_otp_data[j];
		}
		for(j=56;j<=63;j++)
		{
			p_otp->AWB[j-56]=t4k35_otp_data[j];
		}
		return 0;
	  }
	  else
	  {
		CDBG("otp lsc checksum error!\n");
		return -1;
	  }
    }
  }
  if (i < 0) 
  {
    return -1;
    CDBG("No otp lsc data on sensor t4k35\n");
  }
  else 
  {
    return 0;
  }
}
static int32_t t4k35_otp_init_module_info(struct msm_sensor_ctrl_t *s_ctrl, struct t4k35_otp_struct *p_otp)
{
  int i,pos;
  uint16_t check_sum=0x00;
  t4k35_otp_read_enble(s_ctrl, 1);
  t4k35_otp_set_page(s_ctrl, 4);
  t4k35_otp_access(s_ctrl);
  CDBG("data area data:\n");
  t4k35_otp_read_data(s_ctrl, t4k35_otp_data);
  t4k35_otp_set_page(s_ctrl, 10);
  t4k35_otp_access(s_ctrl);
  t4k35_otp_read_data(s_ctrl, t4k35_otp_data_backup);
  t4k35_otp_read_enble(s_ctrl, 0);		
  for(i = 0; i < 64; i++) 
  {
	  t4k35_otp_data[i]=t4k35_otp_data[i]|t4k35_otp_data_backup[i];
  }
  if(t4k35_otp_data[32])
  {
	  pos=32;
  }
  else if(t4k35_otp_data[0])
  {
  	  pos=0;
  }
  else
  {
  	  CDBG("otp no module information!\n");
  	  return -1;
  }
  for(i = pos+2; i <pos+32; i++) 
  {
     check_sum=check_sum+t4k35_otp_data[i];
  }
  if((check_sum&0xFF)==t4k35_otp_data[pos+1])
  {
        int moduleId = t4k35_otp_data[pos + 6];
        CDBG("T4k35 moduleId :%d !\n", moduleId);
	  	CDBG("otp module info checksum ok!\n");
		if((t4k35_otp_data[pos+15]==0x00)&&(t4k35_otp_data[pos+16]==0x00)
			&&(t4k35_otp_data[pos+17]==0x00)&&(t4k35_otp_data[pos+18]==0x00)
			&&(t4k35_otp_data[pos+19]==0x00)&&(t4k35_otp_data[pos+20]==0x00)
			&&(t4k35_otp_data[pos+21]==0x00)&&(t4k35_otp_data[pos+22]==0x00))
			return 0;
		t4k35_r_golden_value=t4k35_otp_data[pos+16]+(t4k35_otp_data[pos+15]<<8);
		t4k35_g_golden_value=(t4k35_otp_data[pos+18]+(t4k35_otp_data[pos+17]<<8)+t4k35_otp_data[pos+20]+(t4k35_otp_data[pos+19]<<8))/2;
		t4k35_b_golden_value=t4k35_otp_data[pos+22]+(t4k35_otp_data[pos+21]<<8);
		return 0;
  }
  else
  {
	CDBG("otp module info checksum error!\n");
	return -1;
  }
}
static int32_t t4k35_otp_init_af(struct msm_sensor_ctrl_t *s_ctrl)
{
  int i,pos;
  uint16_t check_sum=0x00;
  t4k35_otp_read_enble(s_ctrl, 1);
  t4k35_otp_set_page(s_ctrl, 5);
  t4k35_otp_access(s_ctrl);
  CDBG("data area data:\n");
  t4k35_otp_read_data(s_ctrl, t4k35_otp_data);
  t4k35_otp_set_page(s_ctrl, 11);
  t4k35_otp_access(s_ctrl);
  t4k35_otp_read_data(s_ctrl, t4k35_otp_data_backup);
  t4k35_otp_read_enble(s_ctrl, 0);		
  for(i = 0; i < 64; i++) 
  {
	  t4k35_otp_data[i]=t4k35_otp_data[i]|t4k35_otp_data_backup[i];
  }
  if(t4k35_otp_data[24])
  {
	  pos=24;
  }
  else if(t4k35_otp_data[16])
  {
  	  pos=16;
  }
  else if(t4k35_otp_data[8])
  {
  	  pos=8;
  }
  else if(t4k35_otp_data[0])
  {
  	  pos=0;
  }
  else
  {
  	  CDBG("no otp macro AF information!\n");
  	  return -1;
  }
  check_sum=0x00;
  for(i = pos+2; i <pos+8; i++) 
  {
     check_sum=check_sum+t4k35_otp_data[i];
  }
  if((check_sum&0xFF)==t4k35_otp_data[pos+1])
  {
	  	CDBG("otp macro AF checksum ok!\n");
        s_ctrl->af_otp_macro=(t4k35_otp_data[pos+3]<<8)+t4k35_otp_data[pos+4];
        if(s_ctrl->af_otp_macro==0x00)
        {
           s_ctrl->af_otp_macro=t4k35_af_macro_pos;
        }
  }
  else
  {
	CDBG("otp macro AF checksum error!\n");
    s_ctrl->af_otp_macro=t4k35_af_macro_pos;
  }
  if(t4k35_otp_data[56])
  {
	  pos=56;
  }
  else if(t4k35_otp_data[48])
  {
  	  pos=48;
  }
  else if(t4k35_otp_data[40])
  {
  	  pos=40;
  }
  else if(t4k35_otp_data[32])
  {
  	  pos=32;
  }
  else
  {
  	  CDBG("no otp inifity AF information!\n");
  	  return -1;
  }
  check_sum=0x00;
  for(i = pos+2; i <pos+8; i++) 
  {
     check_sum=check_sum+t4k35_otp_data[i];
  }
  if((check_sum&0xFF)==t4k35_otp_data[pos+1])
  {
	  	CDBG("otp inifity AF checksum ok!\n");
        s_ctrl->af_otp_inifity=(t4k35_otp_data[pos+4]<<8)+t4k35_otp_data[pos+5];
        if(s_ctrl->af_otp_inifity==0x00)
        {
           s_ctrl->af_otp_inifity=t4k35_af_inifity_pos;
        }
  }
  else
  {
	CDBG("otp inifity AF checksum error!\n");
    s_ctrl->af_otp_inifity=t4k35_af_inifity_pos;
  }
  return 0;
}
static int32_t t4k35_otp_init_setting(struct msm_sensor_ctrl_t *s_ctrl)
{
    int32_t rc = 0;
	st_t4k35_otp t4k35_data;
	rc=t4k35_otp_init_module_info(s_ctrl, &t4k35_data);
	if(rc==0x00)
	{
	}
    rc=t4k35_otp_init_lsc_awb(s_ctrl, &t4k35_data);
	if(rc==0x00)
	{
		t4k35_update_lsc(s_ctrl, &t4k35_data);
		t4k35_update_awb(s_ctrl, &t4k35_data);
	}
	t4k35_otp_init_af(s_ctrl);
    return rc;
}
#endif
static int msm_sensor_platform_remove(struct platform_device *pdev)
{
	struct msm_sensor_ctrl_t  *s_ctrl;

	pr_err("%s: sensor FREE\n", __func__);

	s_ctrl = g_sctrl[pdev->id];
	if (!s_ctrl) {
		pr_err("%s: sensor device is NULL\n", __func__);
		return 0;
	}

	msm_sensor_free_sensor_data(s_ctrl);
	kfree(s_ctrl->msm_sensor_mutex);
	kfree(s_ctrl->sensor_i2c_client);
	kfree(s_ctrl);
	g_sctrl[pdev->id] = NULL;

	return 0;
}


static const struct of_device_id msm_sensor_driver_dt_match[] = {
	{.compatible = "qcom,camera"},
	{}
};

MODULE_DEVICE_TABLE(of, msm_sensor_driver_dt_match);

static struct platform_driver msm_sensor_platform_driver = {
	.driver = {
		.name = "qcom,camera",
		.owner = THIS_MODULE,
		.of_match_table = msm_sensor_driver_dt_match,
	},
	.remove = msm_sensor_platform_remove,
};

static struct v4l2_subdev_info msm_sensor_driver_subdev_info[] = {
	{
		.code = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt = 1,
		.order = 0,
	},
};

static int32_t msm_sensor_driver_create_i2c_v4l_subdev
			(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	uint32_t session_id = 0;
	struct i2c_client *client = s_ctrl->sensor_i2c_client->client;

	CDBG("%s %s I2c probe succeeded\n", __func__, client->name);
	rc = camera_init_v4l2(&client->dev, &session_id);
	if (rc < 0) {
		pr_err("failed: camera_init_i2c_v4l2 rc %d", rc);
		return rc;
	}
	CDBG("%s rc %d session_id %d\n", __func__, rc, session_id);
	snprintf(s_ctrl->msm_sd.sd.name,
		sizeof(s_ctrl->msm_sd.sd.name), "%s",
		s_ctrl->sensordata->sensor_name);
	v4l2_i2c_subdev_init(&s_ctrl->msm_sd.sd, client,
		s_ctrl->sensor_v4l2_subdev_ops);
	v4l2_set_subdevdata(&s_ctrl->msm_sd.sd, client);
	s_ctrl->msm_sd.sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	media_entity_init(&s_ctrl->msm_sd.sd.entity, 0, NULL, 0);
	s_ctrl->msm_sd.sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV;
	s_ctrl->msm_sd.sd.entity.group_id = MSM_CAMERA_SUBDEV_SENSOR;
	s_ctrl->msm_sd.sd.entity.name =	s_ctrl->msm_sd.sd.name;
	s_ctrl->sensordata->sensor_info->session_id = session_id;
	s_ctrl->msm_sd.close_seq = MSM_SD_CLOSE_2ND_CATEGORY | 0x3;
	msm_sd_register(&s_ctrl->msm_sd);
	CDBG("%s:%d\n", __func__, __LINE__);
	return rc;
}

static int32_t msm_sensor_driver_create_v4l_subdev
			(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	uint32_t session_id = 0;

	rc = camera_init_v4l2(&s_ctrl->pdev->dev, &session_id);
	if (rc < 0) {
		pr_err("failed: camera_init_v4l2 rc %d", rc);
		return rc;
	}
	CDBG("rc %d session_id %d", rc, session_id);
	s_ctrl->sensordata->sensor_info->session_id = session_id;

	/* Create /dev/v4l-subdevX device */
	v4l2_subdev_init(&s_ctrl->msm_sd.sd, s_ctrl->sensor_v4l2_subdev_ops);
	snprintf(s_ctrl->msm_sd.sd.name, sizeof(s_ctrl->msm_sd.sd.name), "%s",
		s_ctrl->sensordata->sensor_name);
	v4l2_set_subdevdata(&s_ctrl->msm_sd.sd, s_ctrl->pdev);
	s_ctrl->msm_sd.sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	media_entity_init(&s_ctrl->msm_sd.sd.entity, 0, NULL, 0);
	s_ctrl->msm_sd.sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV;
	s_ctrl->msm_sd.sd.entity.group_id = MSM_CAMERA_SUBDEV_SENSOR;
	s_ctrl->msm_sd.sd.entity.name = s_ctrl->msm_sd.sd.name;
	s_ctrl->msm_sd.close_seq = MSM_SD_CLOSE_2ND_CATEGORY | 0x3;
	msm_sd_register(&s_ctrl->msm_sd);
	msm_sensor_v4l2_subdev_fops = v4l2_subdev_fops;
#ifdef CONFIG_COMPAT
	msm_sensor_v4l2_subdev_fops.compat_ioctl32 =
		msm_sensor_subdev_fops_ioctl;
#endif
	s_ctrl->msm_sd.sd.devnode->fops =
		&msm_sensor_v4l2_subdev_fops;

	return rc;
}

static int32_t msm_sensor_fill_eeprom_subdevid_by_name(
				struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	const char *eeprom_name;
	struct device_node *src_node = NULL;
	uint32_t val = 0, count = 0, eeprom_name_len;
	int i;
	int32_t *eeprom_subdev_id;
	struct  msm_sensor_info_t *sensor_info;
	struct device_node *of_node = s_ctrl->of_node;
	const void *p;

	if (!s_ctrl->sensordata->eeprom_name || !of_node)
		return -EINVAL;

	eeprom_name_len = strlen(s_ctrl->sensordata->eeprom_name);
	if (eeprom_name_len >= MAX_SENSOR_NAME)
		return -EINVAL;

	sensor_info = s_ctrl->sensordata->sensor_info;
	eeprom_subdev_id = &sensor_info->subdev_id[SUB_MODULE_EEPROM];
	/*
	 * string for eeprom name is valid, set sudev id to -1
	 *  and try to found new id
	 */
	*eeprom_subdev_id = -1;

	if (0 == eeprom_name_len)
		return 0;

	CDBG("Try to find eeprom subdev for %s\n",
			s_ctrl->sensordata->eeprom_name);
	p = of_get_property(of_node, "qcom,eeprom-src", &count);
	if (!p || !count)
		return 0;

	count /= sizeof(uint32_t);
	for (i = 0; i < count; i++) {
		eeprom_name = NULL;
		src_node = of_parse_phandle(of_node, "qcom,eeprom-src", i);
		if (!src_node) {
			pr_err("eeprom src node NULL\n");
			continue;
		}
		rc = of_property_read_string(src_node, "qcom,eeprom-name",
			&eeprom_name);
		if (rc < 0) {
			pr_err("failed\n");
			of_node_put(src_node);
			continue;
		}
		if (strcmp(eeprom_name, s_ctrl->sensordata->eeprom_name))
			continue;

		rc = of_property_read_u32(src_node, "cell-index", &val);

		CDBG("%s qcom,eeprom cell index %d, rc %d\n", __func__,
			val, rc);
		if (rc < 0) {
			pr_err("failed\n");
			of_node_put(src_node);
			continue;
		}

		*eeprom_subdev_id = val;
		CDBG("Done. Eeprom subdevice id is %d\n", val);
		of_node_put(src_node);
		src_node = NULL;
		break;
	}

	return rc;
}

static int32_t msm_sensor_fill_actuator_subdevid_by_name(
				struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	struct device_node *src_node = NULL;
	uint32_t val = 0, actuator_name_len;
	int32_t *actuator_subdev_id;
	struct  msm_sensor_info_t *sensor_info;
	struct device_node *of_node = s_ctrl->of_node;

	if (!s_ctrl->sensordata->actuator_name || !of_node)
		return -EINVAL;

	actuator_name_len = strlen(s_ctrl->sensordata->actuator_name);
	if (actuator_name_len >= MAX_SENSOR_NAME)
		return -EINVAL;

	sensor_info = s_ctrl->sensordata->sensor_info;
	actuator_subdev_id = &sensor_info->subdev_id[SUB_MODULE_ACTUATOR];
	/*
	 * string for actuator name is valid, set sudev id to -1
	 * and try to found new id
	 */
	*actuator_subdev_id = -1;

	if (0 == actuator_name_len)
		return 0;

	src_node = of_parse_phandle(of_node, "qcom,actuator-src", 0);
	if (!src_node) {
		CDBG("%s:%d src_node NULL\n", __func__, __LINE__);
	} else {
		rc = of_property_read_u32(src_node, "cell-index", &val);
		CDBG("%s qcom,actuator cell index %d, rc %d\n", __func__,
			val, rc);
		if (rc < 0) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			return -EINVAL;
		}
		*actuator_subdev_id = val;
		of_node_put(src_node);
		src_node = NULL;
	}

	return rc;
}

static int32_t msm_sensor_fill_ois_subdevid_by_name(
				struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	struct device_node *src_node = NULL;
	uint32_t val = 0, ois_name_len;
	int32_t *ois_subdev_id;
	struct  msm_sensor_info_t *sensor_info;
	struct device_node *of_node = s_ctrl->of_node;

	if (!s_ctrl->sensordata->ois_name || !of_node)
		return -EINVAL;

	ois_name_len = strlen(s_ctrl->sensordata->ois_name);
	if (ois_name_len >= MAX_SENSOR_NAME)
		return -EINVAL;

	sensor_info = s_ctrl->sensordata->sensor_info;
	ois_subdev_id = &sensor_info->subdev_id[SUB_MODULE_OIS];
	/*
	 * string for ois name is valid, set sudev id to -1
	 * and try to found new id
	 */
	*ois_subdev_id = -1;

	if (0 == ois_name_len)
		return 0;

	src_node = of_parse_phandle(of_node, "qcom,ois-src", 0);
	if (!src_node) {
		CDBG("%s:%d src_node NULL\n", __func__, __LINE__);
	} else {
		rc = of_property_read_u32(src_node, "cell-index", &val);
		CDBG("%s qcom,ois cell index %d, rc %d\n", __func__,
			val, rc);
		if (rc < 0) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			return -EINVAL;
		}
		*ois_subdev_id = val;
		of_node_put(src_node);
		src_node = NULL;
	}

	return rc;
}

static int32_t msm_sensor_fill_slave_info_init_params(
	struct msm_camera_sensor_slave_info *slave_info,
	struct msm_sensor_info_t *sensor_info)
{
	struct msm_sensor_init_params *sensor_init_params;
	if (!slave_info ||  !sensor_info)
		return -EINVAL;

	if (!slave_info->is_init_params_valid)
		return 0;

	sensor_init_params = &slave_info->sensor_init_params;
	if (INVALID_CAMERA_B != sensor_init_params->position)
		sensor_info->position =
			sensor_init_params->position;

	if (SENSOR_MAX_MOUNTANGLE > sensor_init_params->sensor_mount_angle) {
		sensor_info->sensor_mount_angle =
			sensor_init_params->sensor_mount_angle;
		sensor_info->is_mount_angle_valid = 1;
	}

	if (CAMERA_MODE_INVALID != sensor_init_params->modes_supported)
		sensor_info->modes_supported =
			sensor_init_params->modes_supported;

	return 0;
}


static int32_t msm_sensor_validate_slave_info(
	struct msm_sensor_info_t *sensor_info)
{
	if (INVALID_CAMERA_B == sensor_info->position) {
		sensor_info->position = BACK_CAMERA_B;
		CDBG("%s:%d Set default sensor position\n",
			__func__, __LINE__);
	}
	if (CAMERA_MODE_INVALID == sensor_info->modes_supported) {
		sensor_info->modes_supported = CAMERA_MODE_2D_B;
		CDBG("%s:%d Set default sensor modes_supported\n",
			__func__, __LINE__);
	}
	if (SENSOR_MAX_MOUNTANGLE <= sensor_info->sensor_mount_angle) {
		sensor_info->sensor_mount_angle = 0;
		CDBG("%s:%d Set default sensor mount angle\n",
			__func__, __LINE__);
		sensor_info->is_mount_angle_valid = 1;
	}
	return 0;
}

#ifdef CONFIG_COMPAT
static int32_t msm_sensor_get_pw_settings_compat(
	struct msm_sensor_power_setting *ps,
	struct msm_sensor_power_setting *us_ps, uint32_t size)
{
	int32_t rc = 0, i = 0;
	struct msm_sensor_power_setting32 *ps32 =
		kzalloc(sizeof(*ps32) * size, GFP_KERNEL);

	if (!ps32) {
		pr_err("failed: no memory ps32");
		return -ENOMEM;
	}
	if (copy_from_user(ps32, (void *)us_ps, sizeof(*ps32) * size)) {
		pr_err("failed: copy_from_user");
		kfree(ps32);
		return -EFAULT;
	}
	for (i = 0; i < size; i++) {
		ps[i].config_val = ps32[i].config_val;
		ps[i].delay = ps32[i].delay;
		ps[i].seq_type = ps32[i].seq_type;
		ps[i].seq_val = ps32[i].seq_val;
	}
	kfree(ps32);
	return rc;
}
#endif

static int32_t msm_sensor_create_pd_settings(void *setting,
	struct msm_sensor_power_setting *pd, uint32_t size_down,
	struct msm_sensor_power_setting *pu)
{
	int32_t rc = 0;
	int c, end;
	struct msm_sensor_power_setting pd_tmp;

	pr_err("Generating power_down_setting");

#ifdef CONFIG_COMPAT
	if (is_compat_task()) {
		int i = 0;
		struct msm_sensor_power_setting32 *power_setting_iter =
		(struct msm_sensor_power_setting32 *)compat_ptr((
		(struct msm_camera_sensor_slave_info32 *)setting)->
		power_setting_array.power_setting);

		for (i = 0; i < size_down; i++) {
			pd[i].config_val = power_setting_iter[i].config_val;
			pd[i].delay = power_setting_iter[i].delay;
			pd[i].seq_type = power_setting_iter[i].seq_type;
			pd[i].seq_val = power_setting_iter[i].seq_val;
		}
	} else
#endif
	{
		if (copy_from_user(pd, (void *)pu, sizeof(*pd) * size_down)) {
			pr_err("failed: copy_from_user");
			return -EFAULT;
		}
	}
	/* reverse */
	end = size_down - 1;
	for (c = 0; c < size_down/2; c++) {
		pd_tmp = pd[c];
		pd[c] = pd[end];
		pd[end] = pd_tmp;
		end--;
	}
	return rc;
}

static int32_t msm_sensor_get_power_down_settings(void *setting,
	struct msm_camera_sensor_slave_info *slave_info,
	struct msm_camera_power_ctrl_t *power_info)
{
	int32_t rc = 0;
	uint16_t size_down = 0;
	uint16_t i = 0;
	struct msm_sensor_power_setting *pd = NULL;

	/* DOWN */
	size_down = slave_info->power_setting_array.size_down;
	if (!size_down || size_down > MAX_POWER_CONFIG)
		size_down = slave_info->power_setting_array.size;
	/* Validate size_down */
	if (size_down > MAX_POWER_CONFIG) {
		pr_err("failed: invalid size_down %d", size_down);
		return -EINVAL;
	}
	/* Allocate memory for power down setting */
	pd = kzalloc(sizeof(*pd) * size_down, GFP_KERNEL);
	if (!pd) {
		pr_err("failed: no memory power_setting %p", pd);
		return -EFAULT;
	}

	if (slave_info->power_setting_array.power_down_setting) {
#ifdef CONFIG_COMPAT
		if (is_compat_task()) {
			rc = msm_sensor_get_pw_settings_compat(
				pd, slave_info->power_setting_array.
				power_down_setting, size_down);
			if (rc < 0) {
				pr_err("failed");
				kfree(pd);
				return -EFAULT;
			}
		} else
#endif
		if (copy_from_user(pd, (void *)slave_info->power_setting_array.
				power_down_setting, sizeof(*pd) * size_down)) {
			pr_err("failed: copy_from_user");
			kfree(pd);
			return -EFAULT;
		}
	} else {

		rc = msm_sensor_create_pd_settings(setting, pd, size_down,
			slave_info->power_setting_array.power_setting);
		if (rc < 0) {
			pr_err("failed");
			kfree(pd);
			return -EFAULT;
		}
	}

	/* Fill power down setting and power down setting size */
	power_info->power_down_setting = pd;
	power_info->power_down_setting_size = size_down;

	/* Print power setting */
	for (i = 0; i < size_down; i++) {
		CDBG("DOWN seq_type %d seq_val %d config_val %ld delay %d",
			pd[i].seq_type, pd[i].seq_val,
			pd[i].config_val, pd[i].delay);
	}
	return rc;
}

static int32_t msm_sensor_get_power_up_settings(void *setting,
	struct msm_camera_sensor_slave_info *slave_info,
	struct msm_camera_power_ctrl_t *power_info)
{
	int32_t rc = 0;
	uint16_t size = 0;
	uint16_t i = 0;
	struct msm_sensor_power_setting *pu = NULL;

	size = slave_info->power_setting_array.size;

	/* Validate size */
	if ((size == 0) || (size > MAX_POWER_CONFIG)) {
		pr_err("failed: invalid power_setting size_up = %d\n", size);
		return -EINVAL;
	}

	/* Allocate memory for power up setting */
	pu = kzalloc(sizeof(*pu) * size, GFP_KERNEL);
	if (!pu) {
		pr_err("failed: no memory power_setting %p", pu);
		return -ENOMEM;
	}

#ifdef CONFIG_COMPAT
	if (is_compat_task()) {
		rc = msm_sensor_get_pw_settings_compat(pu,
			slave_info->power_setting_array.
				power_setting, size);
		if (rc < 0) {
			pr_err("failed");
			kfree(pu);
			return -EFAULT;
		}
	} else
#endif
	{
		if (copy_from_user(pu,
			(void *)slave_info->power_setting_array.power_setting,
			sizeof(*pu) * size)) {
			pr_err("failed: copy_from_user");
			kfree(pu);
			return -EFAULT;
		}
	}

	/* Print power setting */
	for (i = 0; i < size; i++) {
		CDBG("UP seq_type %d seq_val %d config_val %ld delay %d",
			pu[i].seq_type, pu[i].seq_val,
			pu[i].config_val, pu[i].delay);
	}


	/* Fill power up setting and power up setting size */
	power_info->power_setting = pu;
	power_info->power_setting_size = size;

	return rc;
}

static int32_t msm_sensor_get_power_settings(void *setting,
	struct msm_camera_sensor_slave_info *slave_info,
	struct msm_camera_power_ctrl_t *power_info)
{
	int32_t rc = 0;

	rc = msm_sensor_get_power_up_settings(setting, slave_info, power_info);
	if (rc < 0) {
		pr_err("failed");
		return -EINVAL;
	}

	rc = msm_sensor_get_power_down_settings(setting, slave_info,
		power_info);
	if (rc < 0) {
		pr_err("failed");
		return -EINVAL;
	}
	return rc;
}

static void msm_sensor_fill_sensor_info(struct msm_sensor_ctrl_t *s_ctrl,
	struct msm_sensor_info_t *sensor_info, char *entity_name)
{
	uint32_t i;

	if (!s_ctrl || !sensor_info) {
		pr_err("%s:failed\n", __func__);
		return;
	}

	strlcpy(sensor_info->sensor_name, s_ctrl->sensordata->sensor_name,
		MAX_SENSOR_NAME);

	sensor_info->session_id = s_ctrl->sensordata->sensor_info->session_id;

	s_ctrl->sensordata->sensor_info->subdev_id[SUB_MODULE_SENSOR] =
		s_ctrl->sensordata->sensor_info->session_id;
	for (i = 0; i < SUB_MODULE_MAX; i++) {
		sensor_info->subdev_id[i] =
			s_ctrl->sensordata->sensor_info->subdev_id[i];
		sensor_info->subdev_intf[i] =
			s_ctrl->sensordata->sensor_info->subdev_intf[i];
	}

	sensor_info->is_mount_angle_valid =
		s_ctrl->sensordata->sensor_info->is_mount_angle_valid;
	sensor_info->sensor_mount_angle =
		s_ctrl->sensordata->sensor_info->sensor_mount_angle;
	sensor_info->modes_supported =
		s_ctrl->sensordata->sensor_info->modes_supported;
	sensor_info->position =
		s_ctrl->sensordata->sensor_info->position;

	strlcpy(entity_name, s_ctrl->msm_sd.sd.entity.name, MAX_SENSOR_NAME);
}
static int front_camera_id_read_proc(struct seq_file *m, void *v)
{
	return seq_printf(m, "%s\n", g_front_camera_status);
}


static int front_camera_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, front_camera_id_read_proc, NULL);
}
static int back_camera_id_read_proc(struct seq_file *m, void *v)
{
	return seq_printf(m, "%s\n", g_back_camera_status);
}
static int back_camera_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, back_camera_id_read_proc, NULL);
}

static const struct file_operations g_back_camera_proc_ops = 
{
	.open		= back_camera_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};
static const struct file_operations g_front_camera_proc_ops = 
{
	.open		= front_camera_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,

};

int32_t back_camera_proc_file(struct msm_sensor_ctrl_t *s_ctrl)
{
    struct proc_dir_entry *proc_file = NULL;
    pr_err("zte: back_camera_proc_file enter,module name:%s\n",s_ctrl->sensordata->sensor_name);

	if(!strcmp("t4k35",s_ctrl->sensordata->sensor_name))
    {
        sprintf(g_back_camera_status,"%s","BACK Camera ID: t4k35 8M");
    }
	if(!strcmp("imx214",s_ctrl->sensordata->sensor_name))
    {
        sprintf(g_back_camera_status,"%s","BACK Camera ID: imx214 13M");
    }
   	proc_file = proc_create("driver/camera_id_back",0644,NULL,&g_back_camera_proc_ops);
	if(!proc_file)
    {
        printk(KERN_INFO "camera_proc_file error!\r\n");
        return -1;
    }
    g_if_back_camera_id_read = true;
    return 0;
}
int32_t front_camera_proc_file(struct msm_sensor_ctrl_t *s_ctrl)
{
    struct proc_dir_entry *proc_file  = NULL;
    pr_err("zte: front_camera_proc_file,module name:%s\n",s_ctrl->sensordata->sensor_name);

    if(!strcmp("s5k5e2",s_ctrl->sensordata->sensor_name))
    {
        sprintf(g_front_camera_status,"%s","FRONT Camera ID: s5k5e2 5M");
    }
    proc_file = proc_create("driver/camera_id_front",0644,NULL, &g_front_camera_proc_ops);
    if(!proc_file)
    {
        printk(KERN_INFO "camera_proc_file error!\r\n");
        return -1;
    }
    g_if_front_camera_id_read = true;
    return 0;
}

//zoupeng add imx 214 otp start
struct msm_eeprom_ctrl_t 
	{	
	struct platform_device *pdev;	
	struct mutex *eeprom_mutex;	
	struct v4l2_subdev sdev;	
	struct v4l2_subdev_ops *eeprom_v4l2_subdev_ops;	
	enum msm_camera_device_type_t eeprom_device_type;	
	struct msm_sd_subdev msm_sd;	
	enum cci_i2c_master_t cci_master;	
	struct msm_camera_i2c_client i2c_client;	
	uint32_t num_bytes;	
	uint8_t *memory_data;	
	uint8_t is_supported;	
	struct msm_eeprom_board_info *eboard_info;	
	uint32_t subdev_id;
	};

static const struct of_device_id msm_eeprom_dt_match[] = 
{
    { .compatible = "qcom,eeprom0" },	
    {}
}
;
/*static int32_t msm_eeprom_platform_remove(struct platform_device *pdev)
{
    return 0;
}*/

static int check_eeprom_id(struct msm_eeprom_ctrl_t *e_ctrl)
{	
    int rc = 0;	
    uint8_t data[4] = {0};  
    CDBG("%s: imx214 read eeprom id ENTRE\n",__func__);	
    e_ctrl->i2c_client.cci_client->sid = 0x50;	
    e_ctrl->i2c_client.addr_type = MSM_CAMERA_I2C_BYTE_ADDR;	
    rc = e_ctrl->i2c_client.i2c_func_tbl->i2c_read_seq(&(e_ctrl->i2c_client), 0,&data[0], 4);	
    if(rc) 
    {		
        pr_err("%s: imx214 read eeprom id error!\n",__func__);
    }	
    else 
    {		
        CDBG("%s: module id low=0x%x, high=0x%x****lens id low=0x%x high=0x%x\n", __func__,data[0], data[1], data[2], data[3]);	
    }	return rc;	
}
static int32_t UpdateAWBLSC(struct msm_eeprom_ctrl_t *e_ctrl)
{
    int rc=0;
    uint16_t imx214_rg ,imx214_bg ,imx214_dig_rg,imx214_dig_gb,imx214_golden_rg,imx214_golden_bg;
    struct msm_sensor_ctrl_t *s_ctrl = NULL;
    uint8_t* data = e_ctrl->memory_data;
    //  uint8_t* lscPointer=e_ctrl->memory_data[16];  //lcs start
    imx214_rg=data[1]*256+data[0];
    imx214_bg=data[3]*256+data[2];
    imx214_golden_rg=data[7]*256+data[6];
    imx214_golden_bg=data[9]*256+data[8];
    //   check_eeprom_id(e_ctrl);
    if(imx214_golden_rg==0)
    {
        printk("this is golden ,return...\n");
        //return rc;
        imx214_golden_rg=511;
        imx214_golden_bg=588;
    }
   imx214_dig_rg=imx214_rg*256/imx214_golden_rg;
   imx214_dig_gb=imx214_bg*256/imx214_golden_bg;
    s_ctrl = g_sctrl[0];
    printk("imx214_rg=%d,imx214_bg=%d,imx214_golden_rg=%d,imx214_golden_bg=%d,imx214_dig_rg=%d,imx214_dig_gb=%d \n",
    	  imx214_rg,imx214_bg ,imx214_golden_rg ,imx214_golden_bg,imx214_dig_rg,imx214_dig_gb);
    //printk("imx214_rg=%d,");
    // s_ctrl->i2c_client.cci_client->sid = 0x20;
    s_ctrl = g_sctrl[0];
    s_ctrl->sensor_i2c_client->addr_type =MSM_CAMERA_I2C_WORD_ADDR;
    //s_ctrl->sensor_i2c_client->cci_client->sid = 0x20;
    s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
      (s_ctrl->sensor_i2c_client),0x0210,(imx214_dig_rg&0x0700)>>8,MSM_CAMERA_I2C_BYTE_DATA);
    s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
      (s_ctrl->sensor_i2c_client),0x0211,imx214_dig_rg&0x00FF,MSM_CAMERA_I2C_BYTE_DATA);
    s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
      (s_ctrl->sensor_i2c_client),0x0212,(imx214_dig_gb&0x0700)>>8,MSM_CAMERA_I2C_BYTE_DATA);
    s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
      (s_ctrl->sensor_i2c_client),0x0213,imx214_dig_gb&0x00FF,MSM_CAMERA_I2C_BYTE_DATA);
    return rc;
}
static int32_t msm_eeprom_config(struct msm_eeprom_ctrl_t *e_ctrl,	void __user *argp)
{
    struct msm_eeprom_cfg_data *cdata = 	(struct msm_eeprom_cfg_data *)argp; 
    int32_t rc = 0; 
    //int j = 0;	
    printk("%s E,cfgtype:%d\n", __func__,cdata->cfgtype);
    switch (cdata->cfgtype) 
    {	
        case CFG_EEPROM_GET_INFO:	
            printk("%s E CFG_EEPROM_GET_INFO\n", __func__);		
            cdata->is_supported = e_ctrl->is_supported;		
            memcpy(cdata->cfg.eeprom_name,			
            e_ctrl->eboard_info->eeprom_name,			
            sizeof(cdata->cfg.eeprom_name));		
        break;
        case CFG_EEPROM_WRITE_DATA:
            printk("msm_eeprom_config CFG_EEPROM_WRITE_DATA zx!\n");
            UpdateAWBLSC(e_ctrl);
        break;
        case CFG_EEPROM_GET_CAL_DATA:		
            printk("%s E CFG_EEPROM_GET_CAL_DATA\n", __func__);		
            cdata->cfg.get_data.num_bytes =			
            e_ctrl->num_bytes;		
            printk("chengjiatest: number bytes = %d\n", cdata->cfg.get_data.num_bytes);		
        break;
        case CFG_EEPROM_READ_CAL_DATA:		
            printk("%s E CFG_EEPROM_READ_CAL_DATA\n", __func__);		
            rc = copy_to_user(cdata->cfg.read_data.dbuffer,			
            e_ctrl->memory_data,			
            cdata->cfg.read_data.num_bytes);
            //printk("chengjiatest: glo = %d\n", cdata->cfg.get_data.num_bytes);	
        default:		break;
    }
    printk("%s X\n", __func__);	
    return rc;
}

static int32_t msm_eeprom_get_subdev_id(	struct msm_eeprom_ctrl_t *e_ctrl, void *arg)
{
    uint32_t *subdev_id = (uint32_t *)arg;	
    printk("%s E\n", __func__);	
    if (!subdev_id) 
    {		
        pr_err("%s failed\n", __func__);		
        return -EINVAL;
    }	
    *subdev_id = e_ctrl->subdev_id;	
    printk("subdev_id %d\n", *subdev_id);	
    printk("%s X\n", __func__);
    return 0;
}
static long msm_eeprom_subdev_ioctl(struct v4l2_subdev *sd,	   unsigned int cmd, void *arg)
{   
    struct msm_eeprom_ctrl_t *e_ctrl = v4l2_get_subdevdata(sd);	
    void __user *argp = (void __user *)arg;	
    printk("%s E\n", __func__);	
    printk("%s:%d a_ctrl %p argp %p,cmd:%d\n", __func__, __LINE__, e_ctrl, argp,cmd);
    switch (cmd) 
    {	
        case VIDIOC_MSM_SENSOR_GET_SUBDEV_ID:		
        return msm_eeprom_get_subdev_id(e_ctrl, argp);
        case VIDIOC_MSM_EEPROM_CFG:		
        return msm_eeprom_config(e_ctrl, argp);
        default:		
        return -ENOIOCTLCMD;	
    }
    printk("%s X\n", __func__);		
    return 0;
}

static struct platform_driver msm_eeprom_platform_driver = 
{
    .driver = 
    {		
        .name = "qcom,eeprom",		
        .owner = THIS_MODULE,		
        .of_match_table = msm_eeprom_dt_match,	
    },	
//.remove = __devexit_p(msm_eeprom_platform_remove),
};
static struct v4l2_subdev_core_ops msm_eeprom_subdev_core_ops =
{	
    .ioctl = msm_eeprom_subdev_ioctl,
};

static struct v4l2_subdev_ops msm_eeprom_subdev_ops1 = 
{	
    .core = &msm_eeprom_subdev_core_ops,
};
DEFINE_MSM_MUTEX(msm_eeprom_mutex);
static struct msm_camera_i2c_fn_t msm_eeprom_cci_func_tbl =
{	
    .i2c_read = msm_camera_cci_i2c_read,	
    .i2c_read_seq = msm_camera_cci_i2c_read_seq,	
    .i2c_write = msm_camera_cci_i2c_write,	
    .i2c_write_table = msm_camera_cci_i2c_write_table,	
    .i2c_write_seq_table = msm_camera_cci_i2c_write_seq_table,	
    .i2c_write_table_w_microdelay =msm_camera_cci_i2c_write_table_w_microdelay,	
    .i2c_util = msm_sensor_cci_i2c_util,	
    .i2c_poll = msm_camera_cci_i2c_poll,
};

static struct eeprom_block_info imx214_block_info[] =
{
    
    //yuxin add OTP infor notes,2014.11.21
    //this array's data sequence must be the same with eeprom_data defined in the eeprom.c
    //from 0x00 to 0x06 stores module ID,Lens ID and Shading Calibration version
    //{0x50, 0x00, 6, MSM_CAMERA_I2C_BYTE_ADDR, MSM_CAMERA_I2C_BYTE_DATA, 0},
    //from 0x06 to 0x10 stores AWB info,R/Gr,B/Gr,Gr/Gb,golden_R/Gr,golden_B/Gr,golden_Gr/Gb
    {0x50, 0x06, 12, MSM_CAMERA_I2C_BYTE_ADDR, MSM_CAMERA_I2C_BYTE_DATA, 0},	
    //from 0x12 to 0x14 stores AF info,Macro position,Infinity position
    {0x50, 0x12, 4, MSM_CAMERA_I2C_BYTE_ADDR, MSM_CAMERA_I2C_BYTE_DATA, 0},
    //0x16 stores LSC size
    //{0x50, 0x16, 2, MSM_CAMERA_I2C_BYTE_ADDR, MSM_CAMERA_I2C_BYTE_DATA, 0},
    //Block2 0xA4 stores ROI 1-51 LSC info
    {0x52, 0x0, 255, MSM_CAMERA_I2C_BYTE_ADDR, MSM_CAMERA_I2C_BYTE_DATA, 0},
     //Block3 0xA6 stores ROI 52-102 LSC info
    {0x53, 0x0, 255, MSM_CAMERA_I2C_BYTE_ADDR, MSM_CAMERA_I2C_BYTE_DATA, 0},
     //Block4 0xA8 stores ROI 103-153 LSC info
     {0x54, 0x0, 255, MSM_CAMERA_I2C_BYTE_ADDR, MSM_CAMERA_I2C_BYTE_DATA, 0},
     //Block5 0xAA stores ROI 154-204 LSC info
    {0x55, 0x0, 255, MSM_CAMERA_I2C_BYTE_ADDR, MSM_CAMERA_I2C_BYTE_DATA, 0},
    //Block6 0xAC stores ROI 205-221 LSC info
    {0x56, 0x0, 85, MSM_CAMERA_I2C_BYTE_ADDR, MSM_CAMERA_I2C_BYTE_DATA, 0},
};

static int msm_eeprom_alloc_memory_map(struct msm_eeprom_ctrl_t *e_ctrl,				      
	struct device_node *of)
{
    int rc = 0;
    int i = 0;
    struct msm_eeprom_board_info *eb = e_ctrl->eboard_info;
    e_ctrl->num_bytes = 0;
    eb->num_blocks = ARRAY_SIZE(imx214_block_info);
    eb->eeprom_map = kzalloc((sizeof(struct eeprom_memory_map_t)* eb->num_blocks), GFP_KERNEL);
    if (!eb->eeprom_map) 
    {		
        pr_err("%s failed line %d\n", __func__, __LINE__);		
        return -ENOMEM;	
    }

    for(i = 0; i < eb->num_blocks; i++) 
    {		
        eb->eeprom_map[i].mem.valid_size = imx214_block_info[i].valid_size;		
        eb->eeprom_map[i].mem.addr = imx214_block_info[i].start_addr;		
        eb->eeprom_map[i].mem.addr_t = imx214_block_info[i].addr_t;		
        eb->eeprom_map[i].mem.data = 0;		
        eb->eeprom_map[i].mem.data_t = imx214_block_info[i].data_t;		
        eb->eeprom_map[i].mem.delay = imx214_block_info[i].delay;		
        e_ctrl->num_bytes += eb->eeprom_map[i].mem.valid_size;
    }
    e_ctrl->memory_data = kzalloc(e_ctrl->num_bytes, GFP_KERNEL);	
    if (!e_ctrl->memory_data) 
    {		
        pr_err("%s failed line %d\n", __func__, __LINE__);		
        rc = -ENOMEM;		
        goto out;	
    }	
    return rc;
    out:	kfree(eb->eeprom_map);	
    return rc;
				
}

//static uint8_t g_sensor_module_info[6];

static void print_eeprom_data(uint8_t* p, int len) 
{
	int index = 0;

	if(len < 0) {
		pr_err("chengjiatest: warning, eeprom length of data is negative\n");
		return;
	}
	CDBG("------");
	for (index = 0; index < len; index++) {
		CDBG("0x%x, ", *(p+index));
		
	}
	CDBG("\n");
	return;
}
static int32_t read_eeprom_memory(struct msm_eeprom_ctrl_t *e_ctrl)
{
    int rc = 0;
    int i = 0;	


    uint8_t *memptr = NULL;
    struct msm_eeprom_board_info *eb_info = NULL;	
    struct eeprom_memory_map_t *emap = NULL;
    CDBG("%s:E\n",__func__);

    if (!e_ctrl)
    {		
        pr_err("%s e_ctrl is NULL\n", __func__);		
        rc = -1;		
        return rc;	
    }
#if 0
    CDBG("%s : read module info start\n",__func__);
    e_ctrl->i2c_client.cci_client->sid = 0x50;
    e_ctrl->i2c_client.addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
    rc = e_ctrl->i2c_client.i2c_func_tbl->i2c_read_seq(
            &(e_ctrl->i2c_client),0x06,
            g_sensor_module_info,12);
    if (rc < 0) 
    {				
        pr_err("%s: read failed\n", __func__);
        return rc;			
    }
    for(i = 0;i<12;i++)
    {
        pr_err("yuxin sensor AWB info %d:0x%x\n",i,(unsigned int)g_sensor_module_info[i]);
    }
    pr_err("zoupeng : read module info end\n");
#endif    
    memptr = e_ctrl->memory_data;	
    eb_info = e_ctrl->eboard_info;	
    emap = eb_info->eeprom_map;
    CDBG("chengjiatest : number blocks = %d\n", eb_info->num_blocks);
    for(i = 0; i < eb_info->num_blocks; i++) 
    {
        e_ctrl->i2c_client.cci_client->sid = imx214_block_info[i].slave_id;

        if (emap[i].mem.valid_size) 
        {			
            e_ctrl->i2c_client.addr_type = emap[i].mem.addr_t;			
            rc = e_ctrl->i2c_client.i2c_func_tbl->i2c_read_seq(				
            &(e_ctrl->i2c_client), emap[i].mem.addr,				
            memptr, emap[i].mem.valid_size);			
            if (rc < 0) 
            {				
                pr_err("%s: read failed\n", __func__);				
                return rc;			
            }
            print_eeprom_data(memptr, emap[i].mem.valid_size);//////print log
            memptr += emap[i].mem.valid_size;		
        }
    }	
    CDBG("%s:X\n",__func__);
    return rc;
}

static int msm_eeprom_open(struct v4l2_subdev *sd,	struct v4l2_subdev_fh *fh) 
{
  return 0;
}
static int msm_eeprom_close(struct v4l2_subdev *sd,	struct v4l2_subdev_fh *fh)
{
  return 0;
}

static const struct v4l2_subdev_internal_ops msm_eeprom_internal_ops = 
{	.open = msm_eeprom_open,	
    .close = msm_eeprom_close,
};

static int32_t msm_eeprom_platform_probe(struct platform_device *pdev)
{
    int32_t rc = 0;  
    struct msm_camera_cci_client *cci_client = NULL;
    struct msm_eeprom_ctrl_t *e_ctrl = NULL;
    struct msm_eeprom_board_info *eb_info = NULL;	
    struct device_node *of_node = pdev->dev.of_node;
    printk("zxtest: eeprom probe!!!\n");
    e_ctrl = kzalloc(sizeof(struct msm_eeprom_ctrl_t), GFP_KERNEL);
    if (!e_ctrl) 
    {		
        pr_err("%s:%d kzalloc failed\n", __func__, __LINE__);		
        return -ENOMEM;
    }
    e_ctrl->eeprom_v4l2_subdev_ops = &msm_eeprom_subdev_ops1;	
    e_ctrl->eeprom_mutex = &msm_eeprom_mutex;
    e_ctrl->is_supported = 0;
    if (!of_node)
    {		
    pr_err("%s dev.of_node NULL\n", __func__);		
    return -EINVAL;	
    }
    pdev->id = 0;	
    e_ctrl->subdev_id = pdev->id;	
    e_ctrl->cci_master = 0;
    e_ctrl->pdev = pdev;	/* Set device type as platform device */	
    e_ctrl->eeprom_device_type = MSM_CAMERA_PLATFORM_DEVICE;	
    e_ctrl->i2c_client.i2c_func_tbl = &msm_eeprom_cci_func_tbl;
    e_ctrl->i2c_client.cci_client = kzalloc(sizeof(		
    struct msm_camera_cci_client), GFP_KERNEL);
    if (!e_ctrl->i2c_client.cci_client)
    {
        pr_err("%s failed no memory\n", __func__);		
        return -ENOMEM;
    }
    e_ctrl->eboard_info = kzalloc(sizeof(		
    struct msm_eeprom_board_info), GFP_KERNEL);
    if (!e_ctrl->eboard_info) 
    {
        pr_err("%s failed line %d\n", __func__, __LINE__);		
        rc = -ENOMEM;		
        goto cciclient_free;
    }
    eb_info = e_ctrl->eboard_info;	
    cci_client = e_ctrl->i2c_client.cci_client; 
    cci_client->cci_subdev = msm_cci_get_subdev();	
    cci_client->cci_i2c_master = e_ctrl->cci_master;	
    cci_client->retries = 3;	
    cci_client->id_map = 0;
    rc = of_property_read_string(of_node, "qcom,eeprom-name",&eb_info->eeprom_name);
    printk("%s qcom,eeprom-name %s, rc %d\n", __func__,eb_info->eeprom_name, rc);
    if (rc < 0) 
    {		
    pr_err("%s failed %d\n", __func__, __LINE__);		
    goto board_free;	

    }
    check_eeprom_id(e_ctrl);
    rc = msm_eeprom_alloc_memory_map(e_ctrl, of_node);
    if (rc)		
    goto board_free;

    rc = read_eeprom_memory(e_ctrl);	
    if (rc < 0) 
    {		
        pr_err("%s read_eeprom_memory failed\n", __func__);		
        goto power_down;	
    }
    pr_err("%s line %d\n", __func__, __LINE__);
    v4l2_subdev_init(&e_ctrl->msm_sd.sd,		
    e_ctrl->eeprom_v4l2_subdev_ops);
    v4l2_set_subdevdata(&e_ctrl->msm_sd.sd, e_ctrl);	
    platform_set_drvdata(pdev, &e_ctrl->msm_sd.sd);
    e_ctrl->msm_sd.sd.internal_ops = &msm_eeprom_internal_ops;	
    e_ctrl->msm_sd.sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
    snprintf(e_ctrl->msm_sd.sd.name,		
    ARRAY_SIZE(e_ctrl->msm_sd.sd.name), "zte_gt24c16");
    media_entity_init(&e_ctrl->msm_sd.sd.entity, 0, NULL, 0);	
    e_ctrl->msm_sd.sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV;
    e_ctrl->msm_sd.sd.entity.group_id = MSM_CAMERA_SUBDEV_EEPROM;	
    msm_sd_register(&e_ctrl->msm_sd);
    e_ctrl->is_supported = 1;	
    return rc;
    power_down:	
    kfree(e_ctrl->memory_data);	
    //kfree(eb_info->eeprom_map);
    board_free:
    kfree(e_ctrl->eboard_info);
    cciclient_free:	
    kfree(e_ctrl->i2c_client.cci_client);	
    return rc;
}
//zoupeng imx214 otp end

/* static function definition */
int32_t msm_sensor_driver_probe(void *setting,
	struct msm_sensor_info_t *probed_info, char *entity_name)
{
	int32_t                              rc = 0;
	struct msm_sensor_ctrl_t            *s_ctrl = NULL;
	struct msm_camera_cci_client        *cci_client = NULL;
	struct msm_camera_sensor_slave_info *slave_info = NULL;
	struct msm_camera_slave_info        *camera_info = NULL;

	unsigned long                        mount_pos = 0;

	/* Validate input parameters */
	if (!setting) {
		pr_err("failed: slave_info %p", setting);
		return -EINVAL;
	}

	/* Allocate memory for slave info */
	slave_info = kzalloc(sizeof(*slave_info), GFP_KERNEL);
	if (!slave_info) {
		pr_err("failed: no memory slave_info %p", slave_info);
		return -ENOMEM;
	}
#ifdef CONFIG_COMPAT
	if (is_compat_task()) {
		struct msm_camera_sensor_slave_info32 setting32;
		if (copy_from_user((void *)&setting32, setting,
			sizeof(setting32))) {
				pr_err("failed: copy_from_user");
				rc = -EFAULT;
				goto free_slave_info;
			}

		strlcpy(slave_info->actuator_name, setting32.actuator_name,
			sizeof(slave_info->actuator_name));

		strlcpy(slave_info->eeprom_name, setting32.eeprom_name,
			sizeof(slave_info->eeprom_name));

		strlcpy(slave_info->sensor_name, setting32.sensor_name,
			sizeof(slave_info->sensor_name));

		strlcpy(slave_info->ois_name, setting32.ois_name,
			sizeof(slave_info->ois_name));

		strlcpy(slave_info->flash_name, setting32.flash_name,
			sizeof(slave_info->flash_name));

		slave_info->addr_type = setting32.addr_type;
		slave_info->camera_id = setting32.camera_id;

		slave_info->i2c_freq_mode = setting32.i2c_freq_mode;
		slave_info->sensor_id_info = setting32.sensor_id_info;

		slave_info->slave_addr = setting32.slave_addr;
		slave_info->power_setting_array.size =
			setting32.power_setting_array.size;
		slave_info->power_setting_array.size_down =
			setting32.power_setting_array.size_down;
		slave_info->power_setting_array.size_down =
			setting32.power_setting_array.size_down;
		slave_info->power_setting_array.power_setting =
			compat_ptr(setting32.power_setting_array.power_setting);
		slave_info->power_setting_array.power_down_setting =
			compat_ptr(setting32.
				power_setting_array.power_down_setting);
		slave_info->is_init_params_valid =
			setting32.is_init_params_valid;
		slave_info->sensor_init_params = setting32.sensor_init_params;
		slave_info->is_flash_supported = setting32.is_flash_supported;
	} else
#endif
	{
		if (copy_from_user(slave_info,
					(void *)setting, sizeof(*slave_info))) {
			pr_err("failed: copy_from_user");
			rc = -EFAULT;
			goto free_slave_info;
		}
	}

	/* Print slave info */
	CDBG("camera id %d", slave_info->camera_id);
	CDBG("slave_addr 0x%x", slave_info->slave_addr);
	CDBG("addr_type %d", slave_info->addr_type);
	CDBG("sensor_id_reg_addr 0x%x",
		slave_info->sensor_id_info.sensor_id_reg_addr);
	CDBG("sensor_id 0x%x", slave_info->sensor_id_info.sensor_id);
	CDBG("size %d", slave_info->power_setting_array.size);
	CDBG("size down %d", slave_info->power_setting_array.size_down);

	if (slave_info->is_init_params_valid) {
		CDBG("position %d",
			slave_info->sensor_init_params.position);
		CDBG("mount %d",
			slave_info->sensor_init_params.sensor_mount_angle);
	}

	/* Validate camera id */
	if (slave_info->camera_id >= MAX_CAMERAS) {
		pr_err("failed: invalid camera id %d max %d",
			slave_info->camera_id, MAX_CAMERAS);
		rc = -EINVAL;
		goto free_slave_info;
	}

	/* Extract s_ctrl from camera id */
	s_ctrl = g_sctrl[slave_info->camera_id];
	if (!s_ctrl) {
		pr_err("failed: s_ctrl %p for camera_id %d", s_ctrl,
			slave_info->camera_id);
		rc = -EINVAL;
		goto free_slave_info;
	}

	CDBG("s_ctrl[%d] %p", slave_info->camera_id, s_ctrl);

	if (s_ctrl->is_probe_succeed == 1) {
		/*
		 * Different sensor on this camera slot has been connected
		 * and probe already succeeded for that sensor. Ignore this
		 * probe
		 */
		if (slave_info->sensor_id_info.sensor_id ==
			s_ctrl->sensordata->cam_slave_info->
				sensor_id_info.sensor_id) {
			pr_err("slot%d: sensor id%d already probed\n",
				slave_info->camera_id,
				s_ctrl->sensordata->cam_slave_info->
					sensor_id_info.sensor_id);
			msm_sensor_fill_sensor_info(s_ctrl,
				probed_info, entity_name);
		} else
			pr_err("slot %d has some other sensor\n",
				slave_info->camera_id);

		rc = 0;
		goto free_slave_info;
	}

	rc = msm_sensor_get_power_settings(setting, slave_info,
		&s_ctrl->sensordata->power_info);
	if (rc < 0) {
		pr_err("failed");
		goto free_slave_info;
	}


	camera_info = kzalloc(sizeof(struct msm_camera_slave_info), GFP_KERNEL);
	if (!camera_info) {
		pr_err("failed: no memory slave_info %p", camera_info);
		goto free_slave_info;

	}

	s_ctrl->sensordata->slave_info = camera_info;

	/* Fill sensor slave info */
	camera_info->sensor_slave_addr = slave_info->slave_addr;
	camera_info->sensor_id_reg_addr =
		slave_info->sensor_id_info.sensor_id_reg_addr;
	camera_info->sensor_id = slave_info->sensor_id_info.sensor_id;

	/* Fill CCI master, slave address and CCI default params */
	if (!s_ctrl->sensor_i2c_client) {
		pr_err("failed: sensor_i2c_client %p",
			s_ctrl->sensor_i2c_client);
		rc = -EINVAL;
		goto free_camera_info;
	}
	/* Fill sensor address type */
	s_ctrl->sensor_i2c_client->addr_type = slave_info->addr_type;
	if (s_ctrl->sensor_i2c_client->client)
		s_ctrl->sensor_i2c_client->client->addr =
			camera_info->sensor_slave_addr;

	cci_client = s_ctrl->sensor_i2c_client->cci_client;
	if (!cci_client) {
		pr_err("failed: cci_client %p", cci_client);
		goto free_camera_info;
	}
	cci_client->cci_i2c_master = s_ctrl->cci_i2c_master;
	cci_client->sid = slave_info->slave_addr >> 1;
	cci_client->retries = 3;
	cci_client->id_map = 0;
	cci_client->i2c_freq_mode = slave_info->i2c_freq_mode;

	/* Parse and fill vreg params for powerup settings */
	rc = msm_camera_fill_vreg_params(
		s_ctrl->sensordata->power_info.cam_vreg,
		s_ctrl->sensordata->power_info.num_vreg,
		s_ctrl->sensordata->power_info.power_setting,
		s_ctrl->sensordata->power_info.power_setting_size);
	if (rc < 0) {
		pr_err("failed: msm_camera_get_dt_power_setting_data rc %d",
			rc);
		goto free_camera_info;
	}

	/* Parse and fill vreg params for powerdown settings*/
	rc = msm_camera_fill_vreg_params(
		s_ctrl->sensordata->power_info.cam_vreg,
		s_ctrl->sensordata->power_info.num_vreg,
		s_ctrl->sensordata->power_info.power_down_setting,
		s_ctrl->sensordata->power_info.power_down_setting_size);
	if (rc < 0) {
		pr_err("failed: msm_camera_fill_vreg_params for PDOWN rc %d",
			rc);
		goto free_camera_info;
	}

	/* Update sensor, actuator and eeprom name in
	*  sensor control structure */
	s_ctrl->sensordata->sensor_name = slave_info->sensor_name;
	s_ctrl->sensordata->eeprom_name = slave_info->eeprom_name;
	s_ctrl->sensordata->actuator_name = slave_info->actuator_name;
	s_ctrl->sensordata->ois_name = slave_info->ois_name;
	/*
	 * Update eeporm subdevice Id by input eeprom name
	 */
	rc = msm_sensor_fill_eeprom_subdevid_by_name(s_ctrl);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto free_camera_info;
	}
	/*
	 * Update actuator subdevice Id by input actuator name
	 */
	rc = msm_sensor_fill_actuator_subdevid_by_name(s_ctrl);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto free_camera_info;
	}

	rc = msm_sensor_fill_ois_subdevid_by_name(s_ctrl);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto free_camera_info;
	}

	/* Power up and probe sensor */
	rc = s_ctrl->func_tbl->sensor_power_up(s_ctrl);
	if (rc < 0) {
		pr_err("%s power up failed", slave_info->sensor_name);
		goto free_camera_info;
	}

	pr_err("%s probe succeeded", slave_info->sensor_name);
        if(CAMERA_0 == slave_info->camera_id && !g_if_back_camera_id_read)
        {
            back_camera_proc_file(s_ctrl);
        }
        if(CAMERA_1 == slave_info->camera_id && !g_if_front_camera_id_read)
        {
            front_camera_proc_file(s_ctrl);
        }
    

	/*
	  Set probe succeeded flag to 1 so that no other camera shall
	 * probed on this slot
	 */
	s_ctrl->is_probe_succeed = 1;

	/*
	 * Update the subdevice id of flash-src based on availability in kernel.
	 */
	if (slave_info->is_flash_supported == 0) {
		s_ctrl->sensordata->sensor_info->
			subdev_id[SUB_MODULE_LED_FLASH] = -1;
	}

	/*
	 * Create /dev/videoX node, comment for now until dummy /dev/videoX
	 * node is created and used by HAL
	 */

	if (s_ctrl->sensor_device_type == MSM_CAMERA_PLATFORM_DEVICE)
		rc = msm_sensor_driver_create_v4l_subdev(s_ctrl);
	else
		rc = msm_sensor_driver_create_i2c_v4l_subdev(s_ctrl);
	if (rc < 0) {
		pr_err("failed: camera creat v4l2 rc %d", rc);
		goto camera_power_down;
	}
	/*zoupeng add imx214 otp  start*/
        platform_driver_probe(&msm_eeprom_platform_driver,	msm_eeprom_platform_probe);
	/*zoupeng add imx214 otp end*/
#if defined(CONFIG_T4K35)
	t4k35_otp_init_setting(s_ctrl);
#endif
#ifdef CONFIG_SENSOR_INFO 
	if(slave_info->camera_id == 0){
    msm_sensorinfo_set_rear_sensor_index(s_ctrl->msm_sd.sd.entity.id);	
  }else{
    msm_sensorinfo_set_front_sensor_index(s_ctrl->msm_sd.sd.entity.id);	
  }
#endif 
	/* Power down */
	s_ctrl->func_tbl->sensor_power_down(s_ctrl);

	rc = msm_sensor_fill_slave_info_init_params(
		slave_info,
		s_ctrl->sensordata->sensor_info);
	if (rc < 0) {
		pr_err("%s Fill slave info failed", slave_info->sensor_name);
		goto free_camera_info;
	}
	rc = msm_sensor_validate_slave_info(s_ctrl->sensordata->sensor_info);
	if (rc < 0) {
		pr_err("%s Validate slave info failed",
			slave_info->sensor_name);
		goto free_camera_info;
	}
	/* Update sensor mount angle and position in media entity flag */
	mount_pos = s_ctrl->sensordata->sensor_info->position << 16;
	mount_pos = mount_pos | ((s_ctrl->sensordata->sensor_info->
		sensor_mount_angle / 90) << 8);
	s_ctrl->msm_sd.sd.entity.flags = mount_pos | MEDIA_ENT_FL_DEFAULT;

	/*Save sensor info*/
	s_ctrl->sensordata->cam_slave_info = slave_info;

	msm_sensor_fill_sensor_info(s_ctrl, probed_info, entity_name);

	return rc;

camera_power_down:
	s_ctrl->func_tbl->sensor_power_down(s_ctrl);
free_camera_info:
	kfree(camera_info);
free_slave_info:
	kfree(slave_info);
	return rc;
}

static int32_t msm_sensor_driver_get_gpio_data(
	struct msm_camera_sensor_board_info *sensordata,
	struct device_node *of_node)
{
	int32_t                      rc = 0, i = 0;
	struct msm_camera_gpio_conf *gconf = NULL;
	uint16_t                    *gpio_array = NULL;
	uint16_t                     gpio_array_size = 0;

	/* Validate input paramters */
	if (!sensordata || !of_node) {
		pr_err("failed: invalid params sensordata %p of_node %p",
			sensordata, of_node);
		return -EINVAL;
	}

	sensordata->power_info.gpio_conf = kzalloc(
			sizeof(struct msm_camera_gpio_conf), GFP_KERNEL);
	if (!sensordata->power_info.gpio_conf) {
		pr_err("failed");
		return -ENOMEM;
	}
	gconf = sensordata->power_info.gpio_conf;

	gpio_array_size = of_gpio_count(of_node);
	CDBG("gpio count %d", gpio_array_size);
	if (!gpio_array_size)
		return 0;

	gpio_array = kzalloc(sizeof(uint16_t) * gpio_array_size, GFP_KERNEL);
	if (!gpio_array) {
		pr_err("failed");
		goto FREE_GPIO_CONF;
	}
	for (i = 0; i < gpio_array_size; i++) {
		gpio_array[i] = of_get_gpio(of_node, i);
		CDBG("gpio_array[%d] = %d", i, gpio_array[i]);
	}

	rc = msm_camera_get_dt_gpio_req_tbl(of_node, gconf, gpio_array,
		gpio_array_size);
	if (rc < 0) {
		pr_err("failed");
		goto FREE_GPIO_CONF;
	}

	rc = msm_camera_init_gpio_pin_tbl(of_node, gconf, gpio_array,
		gpio_array_size);
	if (rc < 0) {
		pr_err("failed");
		goto FREE_GPIO_REQ_TBL;
	}

	kfree(gpio_array);
	return rc;

FREE_GPIO_REQ_TBL:
	kfree(sensordata->power_info.gpio_conf->cam_gpio_req_tbl);
FREE_GPIO_CONF:
	kfree(sensordata->power_info.gpio_conf);
	kfree(gpio_array);
	return rc;
}

static int32_t msm_sensor_driver_get_dt_data(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t                              rc = 0;
	struct msm_camera_sensor_board_info *sensordata = NULL;
	struct device_node                  *of_node = s_ctrl->of_node;
	uint32_t cell_id;

	s_ctrl->sensordata = kzalloc(sizeof(*sensordata), GFP_KERNEL);
	if (!s_ctrl->sensordata) {
		pr_err("failed: no memory");
		return -ENOMEM;
	}

	sensordata = s_ctrl->sensordata;

	/*
	 * Read cell index - this cell index will be the camera slot where
	 * this camera will be mounted
	 */
	rc = of_property_read_u32(of_node, "cell-index", &cell_id);
	if (rc < 0) {
		pr_err("failed: cell-index rc %d", rc);
		goto FREE_SENSOR_DATA;
	}
	s_ctrl->id = cell_id;

	/* Validate cell_id */
	if (cell_id >= MAX_CAMERAS) {
		pr_err("failed: invalid cell_id %d", cell_id);
		rc = -EINVAL;
		goto FREE_SENSOR_DATA;
	}

	/* Check whether g_sctrl is already filled for this cell_id */
	if (g_sctrl[cell_id]) {
		pr_err("failed: sctrl already filled for cell_id %d", cell_id);
		rc = -EINVAL;
		goto FREE_SENSOR_DATA;
	}

	/* Read subdev info */
	rc = msm_sensor_get_sub_module_index(of_node, &sensordata->sensor_info);
	if (rc < 0) {
		pr_err("failed");
		goto FREE_SENSOR_DATA;
	}

	/* Read vreg information */
	rc = msm_camera_get_dt_vreg_data(of_node,
		&sensordata->power_info.cam_vreg,
		&sensordata->power_info.num_vreg);
	if (rc < 0) {
		pr_err("failed: msm_camera_get_dt_vreg_data rc %d", rc);
		goto FREE_SUB_MODULE_DATA;
	}

	/* Read gpio information */
	rc = msm_sensor_driver_get_gpio_data(sensordata, of_node);
	if (rc < 0) {
		pr_err("failed: msm_sensor_driver_get_gpio_data rc %d", rc);
		goto FREE_VREG_DATA;
	}

	/* Get CCI master */
	rc = of_property_read_u32(of_node, "qcom,cci-master",
		&s_ctrl->cci_i2c_master);
	CDBG("qcom,cci-master %d, rc %d", s_ctrl->cci_i2c_master, rc);
	if (rc < 0) {
		/* Set default master 0 */
		s_ctrl->cci_i2c_master = MASTER_0;
		rc = 0;
	}

	/* Get mount angle */
	if (0 > of_property_read_u32(of_node, "qcom,mount-angle",
		&sensordata->sensor_info->sensor_mount_angle)) {
		/* Invalidate mount angle flag */
		sensordata->sensor_info->is_mount_angle_valid = 0;
		sensordata->sensor_info->sensor_mount_angle = 0;
	} else {
		sensordata->sensor_info->is_mount_angle_valid = 1;
	}
	CDBG("%s qcom,mount-angle %d\n", __func__,
		sensordata->sensor_info->sensor_mount_angle);
	if (0 > of_property_read_u32(of_node, "qcom,sensor-position",
		&sensordata->sensor_info->position)) {
		CDBG("%s:%d Invalid sensor position\n", __func__, __LINE__);
		sensordata->sensor_info->position = INVALID_CAMERA_B;
	}
	if (0 > of_property_read_u32(of_node, "qcom,sensor-mode",
		&sensordata->sensor_info->modes_supported)) {
		CDBG("%s:%d Invalid sensor mode supported\n",
			__func__, __LINE__);
		sensordata->sensor_info->modes_supported = CAMERA_MODE_INVALID;
	}
	/* Get vdd-cx regulator */
	/*Optional property, don't return error if absent */
	of_property_read_string(of_node, "qcom,vdd-cx-name",
		&sensordata->misc_regulator);
	CDBG("qcom,misc_regulator %s", sensordata->misc_regulator);

	s_ctrl->set_mclk_23880000 = of_property_read_bool(of_node,
						"qcom,mclk-23880000");

	CDBG("%s qcom,mclk-23880000 = %d\n", __func__,
		s_ctrl->set_mclk_23880000);

	return rc;

FREE_VREG_DATA:
	kfree(sensordata->power_info.cam_vreg);
FREE_SUB_MODULE_DATA:
	kfree(sensordata->sensor_info);
FREE_SENSOR_DATA:
	kfree(sensordata);
	return rc;
}

static int32_t msm_sensor_driver_parse(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t                   rc = 0;

	CDBG("Enter");
	/* Validate input parameters */


	/* Allocate memory for sensor_i2c_client */
	s_ctrl->sensor_i2c_client = kzalloc(sizeof(*s_ctrl->sensor_i2c_client),
		GFP_KERNEL);
	if (!s_ctrl->sensor_i2c_client) {
		pr_err("failed: no memory sensor_i2c_client %p",
			s_ctrl->sensor_i2c_client);
		return -ENOMEM;
	}

	/* Allocate memory for mutex */
	s_ctrl->msm_sensor_mutex = kzalloc(sizeof(*s_ctrl->msm_sensor_mutex),
		GFP_KERNEL);
	if (!s_ctrl->msm_sensor_mutex) {
		pr_err("failed: no memory msm_sensor_mutex %p",
			s_ctrl->msm_sensor_mutex);
		goto FREE_SENSOR_I2C_CLIENT;
	}

	/* Parse dt information and store in sensor control structure */
	rc = msm_sensor_driver_get_dt_data(s_ctrl);
	if (rc < 0) {
		pr_err("failed: rc %d", rc);
		goto FREE_MUTEX;
	}

	/* Initialize mutex */
	mutex_init(s_ctrl->msm_sensor_mutex);

	/* Initilize v4l2 subdev info */
	s_ctrl->sensor_v4l2_subdev_info = msm_sensor_driver_subdev_info;
	s_ctrl->sensor_v4l2_subdev_info_size =
		ARRAY_SIZE(msm_sensor_driver_subdev_info);

	/* Initialize default parameters */
	rc = msm_sensor_init_default_params(s_ctrl);
	if (rc < 0) {
		pr_err("failed: msm_sensor_init_default_params rc %d", rc);
		goto FREE_DT_DATA;
	}

	/* Store sensor control structure in static database */
	g_sctrl[s_ctrl->id] = s_ctrl;
	pr_err("g_sctrl[%d] %p", s_ctrl->id, g_sctrl[s_ctrl->id]);

	return rc;

FREE_DT_DATA:
	kfree(s_ctrl->sensordata->power_info.gpio_conf->gpio_num_info);
	kfree(s_ctrl->sensordata->power_info.gpio_conf->cam_gpio_req_tbl);
	kfree(s_ctrl->sensordata->power_info.gpio_conf);
	kfree(s_ctrl->sensordata->power_info.cam_vreg);
	kfree(s_ctrl->sensordata);
FREE_MUTEX:
	kfree(s_ctrl->msm_sensor_mutex);
FREE_SENSOR_I2C_CLIENT:
	kfree(s_ctrl->sensor_i2c_client);
	return rc;
}

static int32_t msm_sensor_driver_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	struct msm_sensor_ctrl_t *s_ctrl = NULL;

	/* Create sensor control structure */
	s_ctrl = kzalloc(sizeof(*s_ctrl), GFP_KERNEL);
	if (!s_ctrl) {
		pr_err("failed: no memory s_ctrl %p", s_ctrl);
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, s_ctrl);

	/* Initialize sensor device type */
	s_ctrl->sensor_device_type = MSM_CAMERA_PLATFORM_DEVICE;
	s_ctrl->of_node = pdev->dev.of_node;

	rc = msm_sensor_driver_parse(s_ctrl);
	if (rc < 0) {
		pr_err("failed: msm_sensor_driver_parse rc %d", rc);
		goto FREE_S_CTRL;
	}

	/* Fill platform device */
	pdev->id = s_ctrl->id;
	s_ctrl->pdev = pdev;

	/* Fill device in power info */
	s_ctrl->sensordata->power_info.dev = &pdev->dev;
	return rc;
FREE_S_CTRL:
	kfree(s_ctrl);
	return rc;
}

static int32_t msm_sensor_driver_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int32_t rc = 0;
	struct msm_sensor_ctrl_t *s_ctrl;

	CDBG("\n\nEnter: msm_sensor_driver_i2c_probe");
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s %s i2c_check_functionality failed\n",
			__func__, client->name);
		rc = -EFAULT;
		return rc;
	}

	/* Create sensor control structure */
	s_ctrl = kzalloc(sizeof(*s_ctrl), GFP_KERNEL);
	if (!s_ctrl) {
		pr_err("failed: no memory s_ctrl %p", s_ctrl);
		return -ENOMEM;
	}

	i2c_set_clientdata(client, s_ctrl);

	/* Initialize sensor device type */
	s_ctrl->sensor_device_type = MSM_CAMERA_I2C_DEVICE;
	s_ctrl->of_node = client->dev.of_node;

	rc = msm_sensor_driver_parse(s_ctrl);
	if (rc < 0) {
		pr_err("failed: msm_sensor_driver_parse rc %d", rc);
		goto FREE_S_CTRL;
	}

	if (s_ctrl->sensor_i2c_client != NULL) {
		s_ctrl->sensor_i2c_client->client = client;
		s_ctrl->sensordata->power_info.dev = &client->dev;

	}

	return rc;
FREE_S_CTRL:
	kfree(s_ctrl);
	return rc;
}

static int msm_sensor_driver_i2c_remove(struct i2c_client *client)
{
	struct msm_sensor_ctrl_t  *s_ctrl = i2c_get_clientdata(client);

	pr_err("%s: sensor FREE\n", __func__);

	if (!s_ctrl) {
		pr_err("%s: sensor device is NULL\n", __func__);
		return 0;
	}

	g_sctrl[s_ctrl->id] = NULL;
	msm_sensor_free_sensor_data(s_ctrl);
	kfree(s_ctrl->msm_sensor_mutex);
	kfree(s_ctrl->sensor_i2c_client);
	kfree(s_ctrl);

	return 0;
}

static const struct i2c_device_id i2c_id[] = {
	{SENSOR_DRIVER_I2C, (kernel_ulong_t)NULL},
	{ }
};

static struct i2c_driver msm_sensor_driver_i2c = {
	.id_table = i2c_id,
	.probe  = msm_sensor_driver_i2c_probe,
	.remove = msm_sensor_driver_i2c_remove,
	.driver = {
		.name = SENSOR_DRIVER_I2C,
	},
};

static int __init msm_sensor_driver_init(void)
{
	int32_t rc = 0;

	CDBG("Enter");
	rc = platform_driver_probe(&msm_sensor_platform_driver,
		msm_sensor_driver_platform_probe);
	if (!rc) {
		CDBG("probe success");
		return rc;
	} else {
		CDBG("probe i2c");
		rc = i2c_add_driver(&msm_sensor_driver_i2c);
	}

	return rc;
}


static void __exit msm_sensor_driver_exit(void)
{
	CDBG("Enter");
	platform_driver_unregister(&msm_sensor_platform_driver);
	i2c_del_driver(&msm_sensor_driver_i2c);
	return;
}

module_init(msm_sensor_driver_init);
module_exit(msm_sensor_driver_exit);
MODULE_DESCRIPTION("msm_sensor_driver");
MODULE_LICENSE("GPL v2");
