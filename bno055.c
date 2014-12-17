/*
*
****************************************************************************
* Copyright (C) 2013 - 2014 Bosch Sensortec GmbH
*
* File : bno055.c
*
* Date : 2014/12/12
*
* Revision : 2.0.2 $
*
* Usage: Sensor Driver file for BNO055 sensor
*
****************************************************************************
* \section License
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*   Redistributions of source code must retain the above copyright
*   notice, this list of conditions and the following disclaimer.
*
*   Redistributions in binary form must reproduce the above copyright
*   notice, this list of conditions and the following disclaimer in the
*   documentation and/or other materials provided with the distribution.
*
*   Neither the name of the copyright holder nor the names of the
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
* OR CONTRIBUTORS BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
* OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
* PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
*
* The information provided is believed to be accurate and reliable.
* The copyright holder assumes no responsibility
* for the consequences of use
* of such information nor for any infringement of patents or
* other rights of third parties which may result from its use.
* No license is granted by implication or otherwise under any patent or
* patent rights of the copyright holder.
**************************************************************************/

/*********************************************************/
/*				INCLUDES	*/
/*******************************************************/
#include "bno055.h"
/*! file <BNO055 >
    brief <Sensor driver for BNO055> */
/*	STRUCTURE DEFINITIONS	*/
static struct bno055_t *p_bno055;
/*	 LOCAL FUNCTIONS	*/
/*!
 *	@brief
 *	This function is used for initialize
 *	bus read, bus write function pointers,device
 *	address,accel revision id, gyro revision id
 *	mag revision id, software revision id, boot loader
 *	revision id and page id
 *
 *	@param  bno055 - structure pointer
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note While changing the parameter of the bno055_t
 *	consider the following point:
 *	Changing the reference value of the parameter
 *	will changes the local copy or local reference
 *	make sure your changes will not
 *	affect the reference value of the parameter
 *	(Better case don't change the reference value of the parameter)
 */
BNO055_RETURN_FUNCTION_TYPE bno055_init(struct bno055_t *bno055)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = BNO055_ZERO_U8X;
	u8 v_page_zero_u8 = PAGE_ZERO;
	/* Array holding the Software revision id
	*/
	u8 a_SW_ID_u8[ARRAY_SIZE_TWO] = {
	BNO055_ZERO_U8X, BNO055_ZERO_U8X};
	/* stuct parameters are assign to bno055*/
	p_bno055 = bno055;
	/* Write the default page as zero*/
	com_rslt = p_bno055->BNO055_BUS_WRITE_FUNC
	(p_bno055->dev_addr,
	BNO055_PAGE_ID__REG, &v_page_zero_u8, BNO055_ONE_U8X);
	/* Read the chip id of the sensor from page
	zero 0x00 register*/
	com_rslt += p_bno055->BNO055_BUS_READ_FUNC
	(p_bno055->dev_addr,
	BNO055_CHIP_ID__REG, &v_data_u8, BNO055_ONE_U8X);
	p_bno055->chip_id = v_data_u8;
	/* Read the accel revision id from page
	zero 0x01 register*/
	com_rslt += p_bno055->BNO055_BUS_READ_FUNC
	(p_bno055->dev_addr,
	BNO055_ACCEL_REV_ID__REG, &v_data_u8, BNO055_ONE_U8X);
	p_bno055->accel_rev_id = v_data_u8;
	/* Read the mag revision id from page
	zero 0x02 register*/
	com_rslt += p_bno055->BNO055_BUS_READ_FUNC
	(p_bno055->dev_addr,
	BNO055_MAG_REV_ID__REG, &v_data_u8, BNO055_ONE_U8X);
	p_bno055->mag_rev_id = v_data_u8;
	/* Read the gyro revision id from page
	zero 0x02 register*/
	com_rslt += p_bno055->BNO055_BUS_READ_FUNC
	(p_bno055->dev_addr,
	BNO055_GYRO_REV_ID__REG, &v_data_u8, BNO055_ONE_U8X);
	p_bno055->gyro_rev_id = v_data_u8;
	/* Read the boot loader revision from page
	zero 0x06 register*/
	com_rslt += p_bno055->BNO055_BUS_READ_FUNC
	(p_bno055->dev_addr,
	BNO055_BL_REV_ID__REG, &v_data_u8, BNO055_ONE_U8X);
	p_bno055->bl_rev_id = v_data_u8;
	/* Read the software revision id from page
	zero 0x04 and 0x05 register( 2 bytes of data)*/
	com_rslt += p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
	BNO055_SW_REV_ID_LSB__REG,
	a_SW_ID_u8, BNO055_TWO_U8X);
	a_SW_ID_u8[INDEX_ZERO] = BNO055_GET_BITSLICE(
	a_SW_ID_u8[INDEX_ZERO],
	BNO055_SW_REV_ID_LSB);
	p_bno055->sw_rev_id = (u16)
	((((u32)((u8)a_SW_ID_u8[INDEX_ONE])) <<
	BNO055_SHIFT_8_POSITION) | (a_SW_ID_u8[INDEX_ZERO]));
	/* Read the page id from the register 0x07*/
	com_rslt += p_bno055->BNO055_BUS_READ_FUNC
	(p_bno055->dev_addr,
	BNO055_PAGE_ID__REG, &v_data_u8, BNO055_ONE_U8X);
	p_bno055->page_id = v_data_u8;

	return com_rslt;
}
/*!
 *	@brief
 *	This API gives data to the given register and
 *	the data is written in the corresponding register address
 *
 *  @param v_addr_u8 : Address of the register
 *	@param p_data_u8 : Data to be written to the register
 *	@param v_len_u8  : Length of the Data
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
*/
BNO055_RETURN_FUNCTION_TYPE bno055_write_register(u8 v_addr_u8,
u8 *p_data_u8, u8 v_len_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
			/* Write the values of respective given register */
			com_rslt = p_bno055->BNO055_BUS_WRITE_FUNC
			(p_bno055->dev_addr, v_addr_u8, p_data_u8, v_len_u8);
		}
	return com_rslt;
}
/*!
 *	@brief This API reads the data from
 *	the given register address
 *
 *  @param v_addr_u8 : Address of the register
 *  @param p_data_u8 : address of the variable,
 *	read value will be kept
 *  @param v_len_u8  : Length of the data
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_register(u8 v_addr_u8,
u8 *p_data_u8, u8 v_len_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/* Read the value from given register*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr, v_addr_u8, p_data_u8, v_len_u8);
		}
	return com_rslt;
}
/*!
 *	@brief This API reads chip id
 *	from register 0x00 it is a byte of data
 *
 *
 *	@param v_chip_id_u8 : The chip id value 0xA0
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_chip_id(u8 *v_chip_id_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read the chip id*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_CHIP_ID__REG, &v_data_u8, BNO055_ONE_U8X);
			*v_chip_id_u8 = v_data_u8;
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API reads software revision id
 *	from register 0x04 and 0x05 it is a two byte of data
 *
 *	@param v_sw_id_u8 : The SW revision id
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_sw_rev_id(u16 *v_sw_id_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	/* array having the software revision id
	v_data_u8[0] - LSB
	v_data_u8[1] - MSB*/
	u8 v_data_u8[ARRAY_SIZE_TWO] = {BNO055_ZERO_U8X, BNO055_ZERO_U8X};
	s8 v_stat_s8 = ERROR;
	/* Check the struct  p_bno055 is empty*/
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read the two byte value of software
			revision id*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_SW_REV_ID_LSB__REG,
			v_data_u8, BNO055_TWO_U8X);
			v_data_u8[INDEX_ZERO] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_ZERO],
			BNO055_SW_REV_ID_LSB);
			*v_sw_id_u8 = (u16)
			((((u32)((u8)v_data_u8[INDEX_ONE])) <<
			BNO055_SHIFT_8_POSITION)
			| (v_data_u8[INDEX_ZERO]));
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API reads page id
 *	from register 0x07 it is a byte of data
 *
 *
 *	@param v_page_id_u8 : The value of page id
 *
 *	PAGE_ZERO -> 0x00
 *	PAGE_ONE  -> 0x01
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_page_id(u8 *v_page_id_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = BNO055_ZERO_U8X;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/* Read the page id form 0x07*/
		com_rslt = p_bno055->BNO055_BUS_READ_FUNC
		(p_bno055->dev_addr,
		BNO055_PAGE_ID__REG, &v_data_u8, BNO055_ONE_U8X);
		if (com_rslt == SUCCESS) {
			v_data_u8 = BNO055_GET_BITSLICE(v_data_u8,
			BNO055_PAGE_ID);
			*v_page_id_u8 = v_data_u8;
			p_bno055->page_id = v_data_u8;
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to write
 *	the page id register 0x07
 *
 *	@param v_page_id_u8 : The value of page id
 *
 *	PAGE_ZERO -> 0x00
 *	PAGE_ONE  -> 0x01
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_write_page_id(u8 v_page_id_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
			/* Read the current page*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_PAGE_ID__REG, &v_data_u8r, BNO055_ONE_U8X);
			/* Check condition for communication success*/
			if (com_rslt == SUCCESS) {
				v_data_u8r = BNO055_SET_BITSLICE(v_data_u8r,
				BNO055_PAGE_ID, v_page_id_u8);
				/* Write the page id*/
				com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC
				(p_bno055->dev_addr,
				BNO055_PAGE_ID__REG,
				&v_data_u8r, BNO055_ONE_U8X);
				if (com_rslt == SUCCESS)
					p_bno055->page_id = v_page_id_u8;
			} else {
			com_rslt = ERROR;
			}
		}
	return com_rslt;
}
/*!
 *	@brief This API reads accel revision id
 *	from register 0x01 it is a byte of value
 *
 *	@param v_accel_rev_id_u8 : The accel revision id 0xFB
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_accel_rev_id(
u8 *v_accel_rev_id_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read the accel revision id */
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_REV_ID__REG,
			&v_data_u8, BNO055_ONE_U8X);
			*v_accel_rev_id_u8 = v_data_u8;
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API reads mag revision id
 *	from register 0x02 it is a byte of value
 *
 *	@param v_mag_rev_id_u8 : The mag revision id 0x32
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_mag_rev_id(
u8 *v_mag_rev_id_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
			if ((v_stat_s8 == SUCCESS) ||
			(p_bno055->page_id == PAGE_ZERO)) {
				/* Read the mag revision id */
				com_rslt = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_MAG_REV_ID__REG,
				&v_data_u8, BNO055_ONE_U8X);
				*v_mag_rev_id_u8 = v_data_u8;
			} else {
			com_rslt = ERROR;
			}
		}
	return com_rslt;
}
/*!
 *	@brief This API reads gyro revision id
 *	from register 0x03 it is a byte of value
 *
 *	@param v_gyro_rev_id_u8 : The gyro revision id 0xF0
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_gyro_rev_id(
u8 *v_gyro_rev_id_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read the gyro revision id */
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_REV_ID__REG,
			&v_data_u8, BNO055_ONE_U8X);
			*v_gyro_rev_id_u8 = v_data_u8;
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to read boot loader revision id
 *	from register 0x06 it is a byte of value
 *
 *	@param v_bl_rev_id_u8 : The boot loader revision id
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_bl_rev_id(
u8 *v_bl_rev_id_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read the boot loader revision id */
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_BL_REV_ID__REG,
			&v_data_u8, BNO055_ONE_U8X);
			*v_bl_rev_id_u8 = v_data_u8;
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API reads acceleration data X values
 *	from register 0x08 and 0x09 it is a two byte data
 *
 *
 *
 *
 *	@param v_accel_x_s16 : The X raw data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_accel_x(s16 *v_accel_x_s16)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	/* Array holding the accel x value
	v_data_u8[INDEX_ZERO] - LSB
	v_data_u8[INDEX_ONE] - MSB
	*/
	u8 v_data_u8[ARRAY_SIZE_TWO] = {BNO055_ZERO_U8X, BNO055_ZERO_U8X};
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read the accel x axis two byte value*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_DATA_X_LSB_VALUEX__REG,
			v_data_u8, BNO055_TWO_U8X);
			v_data_u8[INDEX_ZERO] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_ZERO],
			BNO055_ACCEL_DATA_X_LSB_VALUEX);
			v_data_u8[INDEX_ONE] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_ONE],
			BNO055_ACCEL_DATA_X_MSB_VALUEX);
			*v_accel_x_s16 = (s16)((((s32)
			(s8)(v_data_u8[INDEX_ONE])) <<
			(BNO055_SHIFT_8_POSITION))
			| (v_data_u8[INDEX_ZERO]));
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API reads acceleration data Y values
 *	from register 0x0A and 0x0B it is a two byte data
 *
 *
 *
 *
 *	@param v_accel_y_s16 : The Y raw data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_accel_y(s16 *v_accel_y_s16)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	/* Array holding the accel y value
	v_data_u8[INDEX_ZERO] - LSB
	v_data_u8[INDEX_ONE] - MSB
	*/
	u8 v_data_u8[ARRAY_SIZE_TWO] = {BNO055_ZERO_U8X, BNO055_ZERO_U8X};
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read the accel y axis two byte value*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_DATA_Y_LSB_VALUEY__REG,
			v_data_u8, BNO055_TWO_U8X);
			v_data_u8[INDEX_ZERO] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_ZERO],
			BNO055_ACCEL_DATA_Y_LSB_VALUEY);
			v_data_u8[INDEX_ONE] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_ONE],
			BNO055_ACCEL_DATA_Y_MSB_VALUEY);
			*v_accel_y_s16 = (s16)((((s32)
			((s8)v_data_u8[INDEX_ONE])) <<
			BNO055_SHIFT_8_POSITION) |
			(v_data_u8[INDEX_ZERO]));
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API reads acceleration data z values
 *	from register 0x0C and 0x0D it is a two byte data
 *
 *
 *
 *
 *	@param v_accel_z_s16 : The z raw data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_accel_z(s16 *v_accel_z_s16)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	/* Array holding the accel z value
	v_data_u8[INDEX_ZERO] - LSB
	v_data_u8[INDEX_ONE] - MSB
	*/
	u8 v_data_u8[ARRAY_SIZE_TWO] = {BNO055_ZERO_U8X, BNO055_ZERO_U8X};
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read the accel z axis two byte value*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_DATA_Z_LSB_VALUEZ__REG,
			v_data_u8, BNO055_TWO_U8X);
			v_data_u8[INDEX_ZERO] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_ZERO],
			BNO055_ACCEL_DATA_Z_LSB_VALUEZ);
			v_data_u8[INDEX_ONE] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_ONE],
			BNO055_ACCEL_DATA_Z_MSB_VALUEZ);
			*v_accel_z_s16 = (s16)((((s32)
			((s8)v_data_u8[INDEX_ONE])) <<
			BNO055_SHIFT_8_POSITION) |
			(v_data_u8[INDEX_ZERO]));
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API reads acceleration data xyz values
 *	from register 0x08 to 0x0D it is a six byte data
 *
 *
 *	@param accel : The value of accel xyz data
 *
 *	Parameter |    result
 *  --------- | -----------------
 *	 x        | The accel x data
 *	 y        | The accel y data
 *	 z        | The accel z data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_accel_xyz(
struct bno055_accel_t *accel)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	/* Array holding the accel xyz value
	v_data_u8[INDEX_ZERO] - x->LSB
	v_data_u8[INDEX_ONE] - x->MSB
	v_data_u8[2] - y->MSB
	v_data_u8[3] - y->MSB
	v_data_u8[4] - z->MSB
	v_data_u8[5] - z->MSB
	*/
	u8 v_data_u8[ARRAY_SIZE_SIX] = {
	BNO055_ZERO_U8X, BNO055_ZERO_U8X,
	BNO055_ZERO_U8X, BNO055_ZERO_U8X,
	BNO055_ZERO_U8X, BNO055_ZERO_U8X};
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_DATA_X_LSB_VALUEX__REG,
			v_data_u8, BNO055_SIX_U8X);
			/* Data X*/
			v_data_u8[INDEX_ZERO] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_ZERO],
			BNO055_ACCEL_DATA_X_LSB_VALUEX);
			v_data_u8[INDEX_ONE] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_ONE],
			BNO055_ACCEL_DATA_X_MSB_VALUEX);
			accel->x = (s16)((((s32)
			((s8)v_data_u8[INDEX_ONE])) <<
			BNO055_SHIFT_8_POSITION)
			| (v_data_u8[INDEX_ZERO]));
			/* Data Y*/
			v_data_u8[INDEX_TWO] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_TWO],
			BNO055_ACCEL_DATA_Y_LSB_VALUEY);
			v_data_u8[INDEX_THREE] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_THREE],
			BNO055_ACCEL_DATA_Y_MSB_VALUEY);
			accel->y = (s16)((((s32)
			((s8)v_data_u8[INDEX_THREE])) <<
			BNO055_SHIFT_8_POSITION)
			| (v_data_u8[INDEX_TWO]));
			/* Data Z*/
			v_data_u8[INDEX_FOUR] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_FOUR],
			BNO055_ACCEL_DATA_Z_LSB_VALUEZ);
			v_data_u8[INDEX_FIVE] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_FIVE],
			BNO055_ACCEL_DATA_Z_MSB_VALUEZ);
			accel->z = (s16)((((s32)
			((s8)v_data_u8[INDEX_FIVE])) <<
			BNO055_SHIFT_8_POSITION)
			| (v_data_u8[INDEX_FOUR]));
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API reads mag data x values
 *	from register 0x0E and 0x0F it is a two byte data
 *
 *
 *
 *
 *	@param v_mag_x_s16 : The x raw data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_mag_x(s16 *v_mag_x_s16)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	/* Array holding the mag x value
	v_data_u8[INDEX_ZERO] - x->LSB
	v_data_u8[INDEX_ONE] - x->MSB
	*/
	u8 v_data_u8[ARRAY_SIZE_TWO] = {BNO055_ZERO_U8X, BNO055_ZERO_U8X};
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/*Read the mag x two bytes of data */
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_MAG_DATA_X_LSB_VALUEX__REG,
			v_data_u8, BNO055_TWO_U8X);
			v_data_u8[INDEX_ZERO] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_ZERO],
			BNO055_MAG_DATA_X_LSB_VALUEX);
			v_data_u8[INDEX_ONE] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_ONE],
			BNO055_MAG_DATA_X_MSB_VALUEX);
			*v_mag_x_s16 = (s16)((((s32)
			((s8)v_data_u8[INDEX_ONE])) <<
			BNO055_SHIFT_8_POSITION) | (v_data_u8[INDEX_ZERO]));
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API reads mag data y values
 *	from register 0x10 and 0x11 it is a two byte data
 *
 *
 *
 *
 *	@param v_mag_y_s16 : The y raw data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_mag_y(s16 *v_mag_y_s16)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	/* Array holding the mag x value
	v_data_u8[INDEX_ZERO] - y->LSB
	v_data_u8[INDEX_ONE] - y->MSB
	*/
	u8 v_data_u8[ARRAY_SIZE_TWO] = {
	BNO055_ZERO_U8X, BNO055_ZERO_U8X};
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/*Read the mag y two bytes of data */
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_MAG_DATA_Y_LSB_VALUEY__REG,
			v_data_u8, BNO055_TWO_U8X);
			v_data_u8[INDEX_ZERO] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_ZERO],
			BNO055_MAG_DATA_Y_LSB_VALUEY);
			v_data_u8[INDEX_ONE] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_ONE],
			BNO055_MAG_DATA_Y_MSB_VALUEY);
			*v_mag_y_s16 = (s16)((((s32)
			((s8)v_data_u8[INDEX_ONE])) <<
			BNO055_SHIFT_8_POSITION) |
			(v_data_u8[INDEX_ZERO]));
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API reads mag data z values
 *	from register 0x12 and 0x13 it is a two byte data
 *
 *
 *
 *
 *	@param v_mag_z_s16 : The z raw data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_mag_z(s16 *v_mag_z_s16)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	/* Array holding the mag x value
	v_data_u8[INDEX_ZERO] - z->LSB
	v_data_u8[INDEX_ONE] - z->MSB
	*/
	u8 v_data_u8[ARRAY_SIZE_TWO] = {
	BNO055_ZERO_U8X, BNO055_ZERO_U8X};
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_MAG_DATA_Z_LSB_VALUEZ__REG,
			v_data_u8, BNO055_TWO_U8X);
			/*Read the mag z two bytes of data */
			v_data_u8[INDEX_ZERO] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_ZERO],
			BNO055_MAG_DATA_Z_LSB_VALUEZ);
			v_data_u8[INDEX_ONE] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_ONE],
			BNO055_MAG_DATA_Z_MSB_VALUEZ);
			*v_mag_z_s16 = (s16)((((s32)
			((s8)v_data_u8[INDEX_ONE])) <<
			BNO055_SHIFT_8_POSITION)
			| (v_data_u8[INDEX_ZERO]));
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API reads mag data xyz values
 *	from register 0x0E to 0x13 it is a six byte data
 *
 *
 *	@param mag : The mag xyz values
 *
 *	Parameter |    result
 *  --------- | -----------------
 *	 x        | The mag x data
 *	 y        | The mag y data
 *	 z        | The mag z data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_mag_xyz(struct bno055_mag_t *mag)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	/* Array holding the mag xyz value
	v_data_u8[INDEX_ZERO] - x->LSB
	v_data_u8[INDEX_ONE] - x->MSB
	v_data_u8[INDEX_TWO] - y->MSB
	v_data_u8[INDEX_THREE] - y->MSB
	v_data_u8[INDEX_FOUR] - z->MSB
	v_data_u8[INDEX_FIVE] - z->MSB
	*/
	u8 v_data_u8[ARRAY_SIZE_SIX] = {
	BNO055_ZERO_U8X, BNO055_ZERO_U8X,
	BNO055_ZERO_U8X, BNO055_ZERO_U8X,
	BNO055_ZERO_U8X, BNO055_ZERO_U8X};
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/*Read the six byte value of mag xyz*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_MAG_DATA_X_LSB_VALUEX__REG,
			v_data_u8, BNO055_SIX_U8X);
			/* Data X*/
			v_data_u8[INDEX_ZERO] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_ZERO],
			BNO055_MAG_DATA_X_LSB_VALUEX);
			v_data_u8[INDEX_ONE] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_ONE],
			BNO055_MAG_DATA_X_MSB_VALUEX);
			mag->x = (s16)((((s32)
			((s8)v_data_u8[INDEX_ONE])) <<
			BNO055_SHIFT_8_POSITION) |
			(v_data_u8[INDEX_ZERO]));
			/* Data Y*/
			v_data_u8[INDEX_TWO] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_TWO],
			BNO055_MAG_DATA_Y_LSB_VALUEY);
			v_data_u8[INDEX_THREE] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_THREE],
			BNO055_MAG_DATA_Y_MSB_VALUEY);
			mag->y = (s16)((((s32)
			((s8)v_data_u8[INDEX_THREE])) <<
			BNO055_SHIFT_8_POSITION) |
			(v_data_u8[INDEX_TWO]));
			/* Data Z*/
			v_data_u8[INDEX_FOUR] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_FOUR],
			BNO055_MAG_DATA_Z_LSB_VALUEZ);
			v_data_u8[INDEX_FIVE] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_FIVE],
			BNO055_MAG_DATA_Z_MSB_VALUEZ);
			mag->z = (s16)((((s32)
			((s8)v_data_u8[INDEX_FIVE])) <<
			BNO055_SHIFT_8_POSITION)
			| (v_data_u8[INDEX_FOUR]));
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API reads gyro data x values
 *	from register 0x14 and 0x15 it is a two byte data
 *
 *
 *
 *
 *	@param v_gyro_x_s16 : The x raw data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_gyro_x(s16 *v_gyro_x_s16)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8[ARRAY_SIZE_TWO] = {
	BNO055_ZERO_U8X, BNO055_ZERO_U8X};
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read the gyro 16 bit x value*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_DATA_X_LSB_VALUEX__REG,
			v_data_u8, BNO055_TWO_U8X);
			v_data_u8[INDEX_ZERO] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_ZERO],
			BNO055_GYRO_DATA_X_LSB_VALUEX);
			v_data_u8[INDEX_ONE] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_ONE],
			BNO055_GYRO_DATA_X_MSB_VALUEX);
			*v_gyro_x_s16 = (s16)((((s32)
			((s8)v_data_u8[INDEX_ONE])) <<
			BNO055_SHIFT_8_POSITION) |
			(v_data_u8[INDEX_ZERO]));
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API reads gyro data y values
 *	from register 0x16 and 0x17 it is a two byte data
 *
 *
 *
 *
 *	@param v_gyro_y_s16 : The y raw data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_gyro_y(s16 *v_gyro_y_s16)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8[ARRAY_SIZE_TWO] = {
	BNO055_ZERO_U8X, BNO055_ZERO_U8X};
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read the value of gyro y */
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_DATA_Y_LSB_VALUEY__REG,
			v_data_u8, BNO055_TWO_U8X);
			v_data_u8[INDEX_ZERO] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_ZERO],
			BNO055_GYRO_DATA_Y_LSB_VALUEY);
			v_data_u8[INDEX_ONE] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_ONE],
			BNO055_GYRO_DATA_Y_MSB_VALUEY);
			*v_gyro_y_s16 = (s16)((((s32)
			((s8)v_data_u8[INDEX_ONE])) <<
			BNO055_SHIFT_8_POSITION)
			| (v_data_u8[INDEX_ZERO]));
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API reads gyro data z values
 *	from register 0x18 and 0x19 it is a two byte data
 *
 *	@param v_gyro_z_s16 : The z raw data
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_gyro_z(s16 *v_gyro_z_s16)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8[ARRAY_SIZE_TWO] = {
	BNO055_ZERO_U8X, BNO055_ZERO_U8X};
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read the gyro z 16 bit value*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_DATA_Z_LSB_VALUEZ__REG,
			v_data_u8, BNO055_TWO_U8X);
			v_data_u8[INDEX_ZERO] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_ZERO],
			BNO055_GYRO_DATA_Z_LSB_VALUEZ);
			v_data_u8[INDEX_ONE] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_ONE],
			BNO055_GYRO_DATA_Z_MSB_VALUEZ);
			*v_gyro_z_s16 = (s16)((((s32)
			((s8)v_data_u8[INDEX_ONE])) <<
			BNO055_SHIFT_8_POSITION)
			| (v_data_u8[INDEX_ZERO]));
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API reads gyro data xyz values
 *	from register 0x14 to 0x19 it is a six byte data
 *
 *
 *	@param gyro : The value of gyro xyz data's
 *
 *	Parameter |    result
 *  --------- | -----------------
 *	 x        | The gyro x data
 *	 y        | The gyro y data
 *	 z        | The gyro z data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_gyro_xyz(struct bno055_gyro_t *gyro)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	/* Array holding the accel xyz value
	v_data_u8[INDEX_ZERO] - x->LSB
	v_data_u8[INDEX_ONE] - x->MSB
	v_data_u8[INDEX_TWO] - y->MSB
	v_data_u8[INDEX_THREE] - y->MSB
	v_data_u8[INDEX_FOUR] - z->MSB
	v_data_u8[INDEX_FIVE] - z->MSB
	*/
	u8 v_data_u8[ARRAY_SIZE_SIX] = {
	BNO055_ZERO_U8X, BNO055_ZERO_U8X,
	BNO055_ZERO_U8X, BNO055_ZERO_U8X,
	BNO055_ZERO_U8X, BNO055_ZERO_U8X};
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read the six bytes data of gyro xyz*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_DATA_X_LSB_VALUEX__REG,
			v_data_u8, BNO055_SIX_U8X);
			/* Data x*/
			v_data_u8[INDEX_ZERO] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_ZERO],
			BNO055_GYRO_DATA_X_LSB_VALUEX);
			v_data_u8[INDEX_ONE] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_ONE],
			BNO055_GYRO_DATA_X_MSB_VALUEX);
			gyro->x = (s16)((((s32)
			((s8)v_data_u8[INDEX_ONE])) <<
			BNO055_SHIFT_8_POSITION) |
			(v_data_u8[INDEX_ZERO]));
			/* Data y*/
			v_data_u8[INDEX_TWO] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_TWO],
			BNO055_GYRO_DATA_Y_LSB_VALUEY);
			v_data_u8[INDEX_THREE] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_THREE],
			BNO055_GYRO_DATA_Y_MSB_VALUEY);
			gyro->y = (s16)((((s32)
			((s8)v_data_u8[INDEX_THREE])) <<
			BNO055_SHIFT_8_POSITION)
			| (v_data_u8[INDEX_TWO]));
			/* Data z*/
			v_data_u8[INDEX_FOUR] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_FOUR],
			BNO055_GYRO_DATA_Z_LSB_VALUEZ);
			v_data_u8[INDEX_FIVE] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_FIVE],
			BNO055_GYRO_DATA_Z_MSB_VALUEZ);
			gyro->z = (s16)((((s32)
			((s8)v_data_u8[INDEX_FIVE])) <<
			BNO055_SHIFT_8_POSITION)
			| (v_data_u8[INDEX_FOUR]));
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API reads gyro data z values
 *	from register 0x1A and 0x1B it is a two byte data
 *
 *	@param v_euler_h_s16 : The raw h data
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_euler_h(s16 *v_euler_h_s16)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	/* Array holding the Euler h value
	v_data_u8[INDEX_ZERO] - h->LSB
	v_data_u8[INDEX_ONE] - h->MSB
	*/
	u8 v_data_u8[ARRAY_SIZE_TWO] = {
	BNO055_ZERO_U8X, BNO055_ZERO_U8X};
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read the eulre heading data*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_EULER_H_LSB_VALUEH__REG,
			v_data_u8, BNO055_TWO_U8X);
			v_data_u8[INDEX_ZERO] = BNO055_GET_BITSLICE
			(v_data_u8[INDEX_ZERO],
			BNO055_EULER_H_LSB_VALUEH);
			v_data_u8[INDEX_ONE] = BNO055_GET_BITSLICE
			(v_data_u8[INDEX_ONE],
			BNO055_EULER_H_MSB_VALUEH);
			*v_euler_h_s16 = (s16)((((s32)
			((s8)v_data_u8[INDEX_ONE])) <<
			BNO055_SHIFT_8_POSITION) |
			(v_data_u8[INDEX_ZERO]));
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API reads Euler data r values
 *	from register 0x1C and 0x1D it is a two byte data
 *
 *	@param v_euler_r_s16 : The raw r data
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_euler_r(s16 *v_euler_r_s16)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	/* Array holding the Euler h value
	v_data_u8[INDEX_ZERO] - r->LSB
	v_data_u8[INDEX_ONE] - r->MSB
	*/
	u8 v_data_u8[ARRAY_SIZE_TWO] = {
	BNO055_ZERO_U8X, BNO055_ZERO_U8X};
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read the Euler roll data*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_EULER_R_LSB_VALUER__REG,
			v_data_u8, BNO055_TWO_U8X);
			v_data_u8[INDEX_ZERO] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_ZERO],
			BNO055_EULER_R_LSB_VALUER);
			v_data_u8[INDEX_ONE] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_ONE],
			BNO055_EULER_R_MSB_VALUER);
			*v_euler_r_s16 = (s16)((((s32)
			((s8)v_data_u8[INDEX_ONE])) <<
			BNO055_SHIFT_8_POSITION)
			| (v_data_u8[INDEX_ZERO]));
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API reads Euler data p values
 *	from register 0x1E and 0x1F it is a two byte data
 *
 *	@param v_euler_p_s16 : The raw p data
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_euler_p(s16 *v_euler_p_s16)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	/* Array holding the Euler p value
	v_data_u8[INDEX_ZERO] - p->LSB
	v_data_u8[INDEX_ONE] - p->MSB
	*/
	u8 v_data_u8[ARRAY_SIZE_TWO] = {
	BNO055_ZERO_U8X, BNO055_ZERO_U8X};
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read the Euler p data*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_EULER_P_LSB_VALUEP__REG,
			v_data_u8, BNO055_TWO_U8X);
			v_data_u8[INDEX_ZERO] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_ZERO],
			BNO055_EULER_P_LSB_VALUEP);
			v_data_u8[INDEX_ONE] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_ONE],
			BNO055_EULER_P_MSB_VALUEP);
			*v_euler_p_s16 = (s16)((((s32)
			((s8)v_data_u8[INDEX_ONE])) <<
			BNO055_SHIFT_8_POSITION)
			| (v_data_u8[INDEX_ZERO]));
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API reads Euler data hrp values
 *	from register 0x1A to 0x1F it is a six byte data
 *
 *
 *	@param euler : The Euler hrp data's
 *
 *	Parameter |    result
 *  --------- | -----------------
 *	 h        | The Euler h data
 *	 r        | The Euler r data
 *	 p        | The Euler p data
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_euler_hrp(
struct bno055_euler_t *euler)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	/* Array holding the Euler hrp value
	v_data_u8[INDEX_ZERO] - h->LSB
	v_data_u8[INDEX_ONE] - h->MSB
	v_data_u8[INDEX_TWO] - r->MSB
	v_data_u8[INDEX_THREE] - r->MSB
	v_data_u8[INDEX_FOUR] - p->MSB
	v_data_u8[INDEX_FIVE] - p->MSB
	*/
	u8 v_data_u8[ARRAY_SIZE_SIX] = {
	BNO055_ZERO_U8X, BNO055_ZERO_U8X,
	BNO055_ZERO_U8X, BNO055_ZERO_U8X,
	BNO055_ZERO_U8X, BNO055_ZERO_U8X};
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read the six byte of Euler hrp data*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_EULER_H_LSB_VALUEH__REG,
			v_data_u8, BNO055_SIX_U8X);
			/* Data h*/
			v_data_u8[INDEX_ZERO] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_ZERO],
			BNO055_EULER_H_LSB_VALUEH);
			v_data_u8[INDEX_ONE] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_ONE],
			BNO055_EULER_H_MSB_VALUEH);
			euler->h = (s16)((((s32)
			((s8)v_data_u8[INDEX_ONE])) <<
			BNO055_SHIFT_8_POSITION) |
			(v_data_u8[INDEX_ZERO]));
			/* Data r*/
			v_data_u8[INDEX_TWO] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_TWO],
			BNO055_EULER_R_LSB_VALUER);
			v_data_u8[INDEX_THREE] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_THREE],
			BNO055_EULER_R_MSB_VALUER);
			euler->r = (s16)((((s32)
			((s8)v_data_u8[INDEX_THREE])) <<
			BNO055_SHIFT_8_POSITION)
			| (v_data_u8[INDEX_TWO]));
			/* Data p*/
			v_data_u8[INDEX_FOUR] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_FOUR],
			BNO055_EULER_P_LSB_VALUEP);
			v_data_u8[INDEX_FIVE] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_FIVE],
			BNO055_EULER_P_MSB_VALUEP);
			euler->p = (s16)((((s32)
			((s8)v_data_u8[INDEX_FIVE])) <<
			BNO055_SHIFT_8_POSITION)
			| (v_data_u8[INDEX_FOUR]));
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API reads quaternion data w values
 *	from register 0x20 and 0x21 it is a two byte data
 *
 *	@param v_quaternion_w_s16 : The raw w data
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_quaternion_w(
s16 *v_quaternion_w_s16)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	/* Array holding the Quaternion w value
	v_data_u8[INDEX_ZERO] - w->LSB
	v_data_u8[INDEX_ONE] - w->MSB
	*/
	u8 v_data_u8[ARRAY_SIZE_TWO] = {
	BNO055_ZERO_U8X, BNO055_ZERO_U8X};
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read the two byte value
			of quaternion w data*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_QUATERNION_DATA_W_LSB_VALUEW__REG,
			v_data_u8, BNO055_TWO_U8X);
			v_data_u8[INDEX_ZERO] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_ZERO],
			BNO055_QUATERNION_DATA_W_LSB_VALUEW);
			v_data_u8[INDEX_ONE] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_ONE],
			BNO055_QUATERNION_DATA_W_MSB_VALUEW);
			*v_quaternion_w_s16 = (s16)((((s32)
			((s8)v_data_u8[INDEX_ONE])) <<
			BNO055_SHIFT_8_POSITION)
			| (v_data_u8[INDEX_ZERO]));
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API reads quaternion data x values
 *	from register 0x22 and 0x23 it is a two byte data
 *
 *	@param v_quaternion_x_s16 : The raw x data
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_quaternion_x(
s16 *v_quaternion_x_s16)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	/* Array holding the quaternion x value
	v_data_u8[INDEX_ZERO] - x->LSB
	v_data_u8[INDEX_ONE] - x->MSB
	*/
	u8 v_data_u8[ARRAY_SIZE_TWO] = {BNO055_ZERO_U8X, BNO055_ZERO_U8X};
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read the two byte value
			of quaternion x data*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_QUATERNION_DATA_X_LSB_VALUEX__REG,
			v_data_u8, BNO055_TWO_U8X);
			v_data_u8[INDEX_ZERO] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_ZERO],
			BNO055_QUATERNION_DATA_X_LSB_VALUEX);
			v_data_u8[INDEX_ONE] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_ONE],
			BNO055_QUATERNION_DATA_X_MSB_VALUEX);
			*v_quaternion_x_s16 = (s16)((((s32)
			((s8)v_data_u8[INDEX_ONE])) <<
			BNO055_SHIFT_8_POSITION) | (v_data_u8[INDEX_ZERO]));
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API reads quaternion data y values
 *	from register 0x24 and 0x25 it is a two byte data
 *
 *	@param v_quaternion_y_s16 : The raw y data
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_quaternion_y(
s16 *v_quaternion_y_s16)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	/* Array holding the quaternion y value
	v_data_u8[INDEX_ZERO] - y->LSB
	v_data_u8[INDEX_ONE] - y->MSB
	*/
	u8 v_data_u8[ARRAY_SIZE_TWO] = {BNO055_ZERO_U8X, BNO055_ZERO_U8X};
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read the two byte value
			of quaternion y data*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_QUATERNION_DATA_Y_LSB_VALUEY__REG,
			v_data_u8, BNO055_TWO_U8X);
			v_data_u8[INDEX_ZERO] = BNO055_GET_BITSLICE
			(v_data_u8[INDEX_ZERO],
			BNO055_QUATERNION_DATA_Y_LSB_VALUEY);
			v_data_u8[INDEX_ONE] = BNO055_GET_BITSLICE
			(v_data_u8[INDEX_ONE],
			BNO055_QUATERNION_DATA_Y_MSB_VALUEY);
			*v_quaternion_y_s16 = (s16)((((s32)
			((s8)v_data_u8[INDEX_ONE])) <<
			BNO055_SHIFT_8_POSITION)
			| (v_data_u8[INDEX_ZERO]));
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API reads quaternion data z values
 *	from register 0x26 and 0x27 it is a two byte data
 *
 *	@param v_quaternion_z_s16 : The raw z data
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_quaternion_z(
s16 *v_quaternion_z_s16)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	/* Array holding the quaternion z value
	v_data_u8[INDEX_ZERO] - z->LSB
	v_data_u8[INDEX_ONE] - z->MSB
	*/
	u8 v_data_u8[ARRAY_SIZE_TWO] = {
	BNO055_ZERO_U8X, BNO055_ZERO_U8X};
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read the two byte value
			of quaternion z data*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_QUATERNION_DATA_Z_LSB_VALUEZ__REG,
			v_data_u8, BNO055_TWO_U8X);
			v_data_u8[INDEX_ZERO] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_ZERO],
			BNO055_QUATERNION_DATA_Z_LSB_VALUEZ);
			v_data_u8[INDEX_ONE] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_ONE],
			BNO055_QUATERNION_DATA_Z_MSB_VALUEZ);
			*v_quaternion_z_s16 = (s16)((((s32)
			((s8)v_data_u8[INDEX_ONE])) <<
			BNO055_SHIFT_8_POSITION)
			| (v_data_u8[INDEX_ZERO]));
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API reads Quaternion data wxyz values
 *	from register 0x20 to 0x27 it is a six byte data
 *
 *
 *	@param quaternion : The value of quaternion wxyz data's
 *
 *	Parameter |    result
 *  --------- | -----------------
 *	 w        | The quaternion w data
 *	 x        | The quaternion x data
 *	 y        | The quaternion y data
 *	 z        | The quaternion z data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_quaternion_wxyz(
struct bno055_quaternion_t *quaternion)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	/* Array holding the quaternion wxyz value
	v_data_u8[INDEX_ZERO] - w->LSB
	v_data_u8[INDEX_ONE] - w->MSB
	v_data_u8[INDEX_TWO] - x->LSB
	v_data_u8[INDEX_THREE] - x->MSB
	v_data_u8[INDEX_FOUR] - y->MSB
	v_data_u8[INDEX_FIVE] - y->MSB
	v_data_u8[6] - z->MSB
	v_data_u8[7] - z->MSB
	*/
	u8 v_data_u8[ARRAY_SIZE_EIGHT] = {
	BNO055_ZERO_U8X, BNO055_ZERO_U8X,
	BNO055_ZERO_U8X, BNO055_ZERO_U8X,
	BNO055_ZERO_U8X, BNO055_ZERO_U8X,
	BNO055_ZERO_U8X, BNO055_ZERO_U8X};
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read the eight byte value
			of quaternion wxyz data*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_QUATERNION_DATA_W_LSB_VALUEW__REG,
			v_data_u8, BNO055_EIGHT_U8X);
			/* Data W*/
			v_data_u8[INDEX_ZERO] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_ZERO],
			BNO055_QUATERNION_DATA_W_LSB_VALUEW);
			v_data_u8[INDEX_ONE] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_ONE],
			BNO055_QUATERNION_DATA_W_MSB_VALUEW);
			quaternion->w = (s16)((((s32)
			((s8)v_data_u8[INDEX_ONE])) <<
			BNO055_SHIFT_8_POSITION) |
			(v_data_u8[INDEX_ZERO]));
			/* Data X*/
			v_data_u8[INDEX_TWO] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_TWO],
			BNO055_QUATERNION_DATA_X_LSB_VALUEX);
			v_data_u8[INDEX_THREE] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_THREE],
			BNO055_QUATERNION_DATA_X_MSB_VALUEX);
			quaternion->x = (s16)((((s32)
			((s8)v_data_u8[INDEX_THREE])) <<
			BNO055_SHIFT_8_POSITION) |
			(v_data_u8[INDEX_TWO]));
			/* Data Y*/
			v_data_u8[INDEX_FOUR] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_FOUR],
			BNO055_QUATERNION_DATA_Y_LSB_VALUEY);
			v_data_u8[INDEX_FIVE] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_FIVE],
			BNO055_QUATERNION_DATA_Y_MSB_VALUEY);
			quaternion->y = (s16)((((s32)
			((s8)v_data_u8[INDEX_FIVE])) <<
			BNO055_SHIFT_8_POSITION) |
			(v_data_u8[INDEX_FOUR]));
			/* Data Z*/
			v_data_u8[INDEX_SIX] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_SIX],
			BNO055_QUATERNION_DATA_Z_LSB_VALUEZ);
			v_data_u8[INDEX_SEVEN] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_SEVEN],
			BNO055_QUATERNION_DATA_Z_MSB_VALUEZ);
			quaternion->z = (s16)((((s32)
			((s8)v_data_u8[INDEX_SEVEN])) <<
			BNO055_SHIFT_8_POSITION) |
			(v_data_u8[INDEX_SIX]));
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API reads Linear accel data x values
 *	from register 0x29 and 0x2A it is a two byte data
 *
 *	@param v_linear_accel_x_s16 : The raw x data
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_linear_accel_x(
s16 *v_linear_accel_x_s16)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	/* Array holding the linear accel x value
	v_data_u8[INDEX_ZERO] - x->LSB
	v_data_u8[INDEX_ONE] - x->MSB
	*/
	u8 v_data_u8[ARRAY_SIZE_TWO] = {
	BNO055_ZERO_U8X, BNO055_ZERO_U8X};
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read the two byte value
			of linear accel x data*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_LINEAR_ACCEL_DATA_X_LSB_VALUEX__REG,
			v_data_u8, BNO055_TWO_U8X);
			v_data_u8[INDEX_ZERO] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_ZERO],
			BNO055_LINEAR_ACCEL_DATA_X_LSB_VALUEX);
			v_data_u8[INDEX_ONE] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_ONE],
			BNO055_LINEAR_ACCEL_DATA_X_MSB_VALUEX);
			*v_linear_accel_x_s16 = (s16)((((s32)
			((s8)v_data_u8[INDEX_ONE])) <<
			BNO055_SHIFT_8_POSITION)
			| (v_data_u8[INDEX_ZERO]));
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API reads Linear accel data x values
 *	from register 0x2B and 0x2C it is a two byte data
 *
 *	@param v_linear_accel_y_s16 : The raw y data
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_linear_accel_y(
s16 *v_linear_accel_y_s16)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	/* Array holding the linear accel y value
	v_data_u8[INDEX_ZERO] - y->LSB
	v_data_u8[INDEX_ONE] - y->MSB
	*/
	u8 v_data_u8[ARRAY_SIZE_TWO] = {
	BNO055_ZERO_U8X, BNO055_ZERO_U8X};
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read the two byte value
			of linear accel y data*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_LINEAR_ACCEL_DATA_Y_LSB_VALUEY__REG,
			v_data_u8, BNO055_TWO_U8X);
			v_data_u8[INDEX_ZERO] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_ZERO],
			BNO055_LINEAR_ACCEL_DATA_Y_LSB_VALUEY);
			v_data_u8[INDEX_ONE] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_ONE],
			BNO055_LINEAR_ACCEL_DATA_Y_MSB_VALUEY);
			*v_linear_accel_y_s16 = (s16)((((s32)
			((s8)v_data_u8[INDEX_ONE])) <<
			BNO055_SHIFT_8_POSITION) | (v_data_u8[INDEX_ZERO]));
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API reads Linear accel data x values
 *	from register 0x2C and 0x2D it is a two byte data
 *
 *	@param v_linear_accel_z_s16 : The raw z data
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_linear_accel_z(
s16 *v_linear_accel_z_s16)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	/* Array holding the linear accel z value
	v_data_u8[INDEX_ZERO] - z->LSB
	v_data_u8[INDEX_ONE] - z->MSB
	*/
	u8 v_data_u8[ARRAY_SIZE_TWO] = {
	BNO055_ZERO_U8X, BNO055_ZERO_U8X};
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read the two byte value
			of linear accel z data*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_LINEAR_ACCEL_DATA_Z_LSB_VALUEZ__REG,
			v_data_u8, BNO055_TWO_U8X);
			v_data_u8[INDEX_ZERO] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_ZERO],
			BNO055_LINEAR_ACCEL_DATA_Z_LSB_VALUEZ);
			v_data_u8[INDEX_ONE] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_ONE],
			BNO055_LINEAR_ACCEL_DATA_Z_MSB_VALUEZ);
			*v_linear_accel_z_s16 = (s16)((((s32)
			((s8)v_data_u8[INDEX_ONE])) <<
			BNO055_SHIFT_8_POSITION) | (v_data_u8[INDEX_ZERO]));
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API reads Linear accel data xyz values
 *	from register 0x28 to 0x2D it is a six byte data
 *
 *
 *	@param linear_accel : The value of linear accel xyz data's
 *
 *	Parameter |    result
 *  --------- | -----------------
 *	 x        | The linear accel x data
 *	 y        | The linear accel y data
 *	 z        | The linear accel z data
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_linear_accel_xyz(
struct bno055_linear_accel_t *linear_accel)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	/* Array holding the linear accel xyz value
	v_data_u8[INDEX_ZERO] - x->LSB
	v_data_u8[INDEX_ONE] - x->MSB
	v_data_u8[INDEX_TWO] - y->MSB
	v_data_u8[INDEX_THREE] - y->MSB
	v_data_u8[INDEX_FOUR] - z->MSB
	v_data_u8[INDEX_FIVE] - z->MSB
	*/
	u8 v_data_u8[ARRAY_SIZE_SIX] = {
	BNO055_ZERO_U8X, BNO055_ZERO_U8X,
	BNO055_ZERO_U8X, BNO055_ZERO_U8X,
	BNO055_ZERO_U8X, BNO055_ZERO_U8X};
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read the six byte value
			of linear accel xyz data*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_LINEAR_ACCEL_DATA_X_LSB_VALUEX__REG,
			v_data_u8, BNO055_SIX_U8X);
			/* Data x*/
			v_data_u8[INDEX_ZERO] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_ZERO],
			BNO055_LINEAR_ACCEL_DATA_X_LSB_VALUEX);
			v_data_u8[INDEX_ONE] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_ONE],
			BNO055_LINEAR_ACCEL_DATA_X_MSB_VALUEX);
			linear_accel->x = (s16)((((s32)
			((s8)v_data_u8[INDEX_ONE])) <<
			BNO055_SHIFT_8_POSITION)
			| (v_data_u8[INDEX_ZERO]));
			/* Data y*/
			v_data_u8[INDEX_TWO] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_TWO],
			BNO055_LINEAR_ACCEL_DATA_Y_LSB_VALUEY);
			v_data_u8[INDEX_THREE] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_THREE],
			BNO055_LINEAR_ACCEL_DATA_Y_MSB_VALUEY);
			linear_accel->y = (s16)((((s32)
			((s8)v_data_u8[INDEX_THREE])) <<
			BNO055_SHIFT_8_POSITION)
			| (v_data_u8[INDEX_TWO]));
			/* Data z*/
			v_data_u8[INDEX_FOUR] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_FOUR],
			BNO055_LINEAR_ACCEL_DATA_Z_LSB_VALUEZ);
			v_data_u8[INDEX_FIVE] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_FIVE],
			BNO055_LINEAR_ACCEL_DATA_Z_MSB_VALUEZ);
			linear_accel->z = (s16)((((s32)
			((s8)v_data_u8[INDEX_FIVE])) <<
			BNO055_SHIFT_8_POSITION)
			| (v_data_u8[INDEX_FOUR]));
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API reads gravity data x values
 *	from register 0x2E and 0x2F it is a two byte data
 *
 *	@param v_gravity_x_s16 : The raw x data
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_gravity_x(
s16 *v_gravity_x_s16)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	/* Array holding the gravity x value
	v_data_u8[INDEX_ZERO] - x->LSB
	v_data_u8[INDEX_ONE] - x->MSB
	*/
	u8 v_data_u8[ARRAY_SIZE_TWO] = {
	BNO055_ZERO_U8X, BNO055_ZERO_U8X};
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read the two byte value
			of gravity x data*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GRAVITY_DATA_X_LSB_VALUEX__REG,
			v_data_u8, BNO055_TWO_U8X);
			v_data_u8[INDEX_ZERO] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_ZERO],
			BNO055_GRAVITY_DATA_X_LSB_VALUEX);
			v_data_u8[INDEX_ONE] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_ONE],
			BNO055_GRAVITY_DATA_X_MSB_VALUEX);
			*v_gravity_x_s16 = (s16)((((s32)
			((s8)v_data_u8[INDEX_ONE])) <<
			BNO055_SHIFT_8_POSITION)
			| (v_data_u8[INDEX_ZERO]));
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API reads gravity data y values
 *	from register 0x30 and 0x31 it is a two byte data
 *
 *	@param v_gravity_y_s16 : The raw y data
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_gravity_y(
s16 *v_gravity_y_s16)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	/* Array holding the gravity y value
	v_data_u8[INDEX_ZERO] - y->LSB
	v_data_u8[INDEX_ONE] - y->MSB
	*/
	u8 v_data_u8[ARRAY_SIZE_TWO] = {
	BNO055_ZERO_U8X, BNO055_ZERO_U8X};
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read the two byte value
			of gravity y data*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GRAVITY_DATA_Y_LSB_VALUEY__REG,
			v_data_u8, BNO055_TWO_U8X);
			v_data_u8[INDEX_ZERO] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_ZERO],
			BNO055_GRAVITY_DATA_Y_LSB_VALUEY);
			v_data_u8[INDEX_ONE] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_ONE],
			BNO055_GRAVITY_DATA_Y_MSB_VALUEY);
			*v_gravity_y_s16 = (s16)((((s32)
			((s8)v_data_u8[INDEX_ONE])) <<
			BNO055_SHIFT_8_POSITION)
			| (v_data_u8[INDEX_ZERO]));
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API reads gravity data z values
 *	from register 0x32 and 0x33 it is a two byte data
 *
 *	@param v_gravity_z_s16 : The raw z data
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_gravity_z(
s16 *v_gravity_z_s16)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	/* Array holding the gravity z value
	v_data_u8[INDEX_ZERO] - z->LSB
	v_data_u8[INDEX_ONE] - z->MSB
	*/
	u8 v_data_u8[ARRAY_SIZE_TWO] = {
	BNO055_ZERO_U8X, BNO055_ZERO_U8X};
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read the two byte value
			of gravity z data*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GRAVITY_DATA_Z_LSB_VALUEZ__REG,
			v_data_u8, BNO055_TWO_U8X);
			v_data_u8[INDEX_ZERO] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_ZERO],
			BNO055_GRAVITY_DATA_Z_LSB_VALUEZ);
			v_data_u8[INDEX_ONE] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_ONE],
			BNO055_GRAVITY_DATA_Z_MSB_VALUEZ);
			*v_gravity_z_s16 = (s16)((((s32)
			((s8)v_data_u8[INDEX_ONE])) <<
			BNO055_SHIFT_8_POSITION)
			| (v_data_u8[INDEX_ZERO]));
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
  *	@brief This API reads gravity data xyz values
 *	from register 0x2E to 0x33 it is a six byte data
 *
 *
 *	@param gravity : The value of gravity xyz data's
 *
 *	Parameter |    result
 *  --------- | -----------------
 *	 x        | The gravity x data
 *	 y        | The gravity y data
 *	 z        | The gravity z data
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_gravity_xyz(
struct bno055_gravity_t *gravity)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	/* Array holding the gravity xyz value
	v_data_u8[INDEX_ZERO] - x->LSB
	v_data_u8[INDEX_ONE] - x->MSB
	v_data_u8[INDEX_TWO] - y->MSB
	v_data_u8[INDEX_THREE] - y->MSB
	v_data_u8[INDEX_FOUR] - z->MSB
	v_data_u8[INDEX_FIVE] - z->MSB
	*/
	u8 v_data_u8[ARRAY_SIZE_SIX] = {
	BNO055_ZERO_U8X, BNO055_ZERO_U8X,
	BNO055_ZERO_U8X, BNO055_ZERO_U8X,
	BNO055_ZERO_U8X, BNO055_ZERO_U8X};
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read the six byte value
			of gravity xyz data*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GRAVITY_DATA_X_LSB_VALUEX__REG,
			v_data_u8, BNO055_SIX_U8X);
			/* Data x*/
			v_data_u8[INDEX_ZERO] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_ZERO],
			BNO055_GRAVITY_DATA_X_LSB_VALUEX);
			v_data_u8[INDEX_ONE] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_ONE],
			BNO055_GRAVITY_DATA_X_MSB_VALUEX);
			gravity->x = (s16)(((s32)
			((s8)v_data_u8[INDEX_ONE]) <<
			BNO055_SHIFT_8_POSITION) |
			(v_data_u8[INDEX_ZERO]));
			/* Data y*/
			v_data_u8[INDEX_TWO] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_TWO],
			BNO055_GRAVITY_DATA_Y_LSB_VALUEY);
			v_data_u8[INDEX_THREE] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_THREE],
			BNO055_GRAVITY_DATA_Y_MSB_VALUEY);
			gravity->y = (s16)((((s32)
			((s8)v_data_u8[INDEX_THREE])) <<
			BNO055_SHIFT_8_POSITION) | (v_data_u8[INDEX_TWO]));
			/* Data z*/
			v_data_u8[INDEX_FOUR] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_FOUR],
			BNO055_GRAVITY_DATA_Z_LSB_VALUEZ);
			v_data_u8[INDEX_FIVE] =
			BNO055_GET_BITSLICE(v_data_u8[INDEX_FIVE],
			BNO055_GRAVITY_DATA_Z_MSB_VALUEZ);
			gravity->z = (s16)((((s32)
			((s8)v_data_u8[INDEX_FIVE])) <<
			BNO055_SHIFT_8_POSITION)
			| (v_data_u8[INDEX_FOUR]));
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API reads temperature values
 *	from register 0x33 it is a byte data
 *
 *	@param v_temp_s8 : The raw temperature data
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_temp_data(s8 *v_temp_s8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read the raw temperature data */
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_TEMP__REG, &v_data_u8, BNO055_ONE_U8X);
			*v_temp_s8 = v_data_u8;
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
#ifdef	BNO055_FLOAT_ENABLE
/*!
 *	@brief This API is used to convert the accel x raw data
 *	to meterpersecseq output as float
 *
 *	@param v_accel_x_f : The accel x meterpersecseq data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_accel_x_msq(
float *v_accel_x_f)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	s16 v_reg_accel_x_s16 = BNO055_ZERO_U8X;
	float v_data_f = BNO055_ZERO_U8X;
	u8 v_accel_unit_u8 = BNO055_ZERO_U8X;
	/* Read the current accel unit and set the
	unit as m/s2 if the unit is in mg*/
	com_rslt = bno055_get_accel_unit(&v_accel_unit_u8);
	if (v_accel_unit_u8 != ACCEL_UNIT_MSQ)
		com_rslt += bno055_set_accel_unit(ACCEL_UNIT_MSQ);
		if (com_rslt == SUCCESS) {
			/* Read the accel raw x data*/
			com_rslt += bno055_read_accel_x(&v_reg_accel_x_s16);
			p_bno055->delay_msec(BNO055_ONE_U8X);
			if (com_rslt == SUCCESS) {
				/* Convert the raw accel x to m/s2*/
				v_data_f =
				(float)(v_reg_accel_x_s16/ACCEL_DIV_MSQ);
				*v_accel_x_f = v_data_f;
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the accel x raw data
 *	to millig output as float
 *
 *	@param v_accel_x_f : The accel x millig data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_accel_x_mg(
float *v_accel_x_f)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	s16 v_reg_accel_x_s16 = BNO055_ZERO_U8X;
	float v_data_f = BNO055_ZERO_U8X;
	u8 v_accel_unit_u8 = BNO055_ZERO_U8X;
	/* Read the current accel unit and set the
	unit as mg if the unit is in m/s2*/
	com_rslt = bno055_get_accel_unit(&v_accel_unit_u8);
	if (v_accel_unit_u8 != ACCEL_UNIT_MG)
		com_rslt += bno055_set_accel_unit(ACCEL_UNIT_MG);
		if (com_rslt == SUCCESS) {
			/* Read the accel raw x data*/
			com_rslt += bno055_read_accel_x(&v_reg_accel_x_s16);
			if (com_rslt == SUCCESS) {
				/* Convert the raw accel x to m/s2*/
				v_data_f =
				(float)(v_reg_accel_x_s16/ACCEL_DIV_MG);
				*v_accel_x_f = v_data_f;
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the accel x raw data
 *	to meterpersecseq output as float
 *
 *	@param v_accel_y_f : The accel y meterpersecseq data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_accel_y_msq(
float *v_accel_y_f)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	s16 v_reg_accel_y_s16 = BNO055_ZERO_U8X;
	float v_data_f = BNO055_ZERO_U8X;
	u8 v_accel_unit_u8 = BNO055_ZERO_U8X;
	/* Read the current accel unit and set the
	unit as m/s2 if the unit is in mg*/
	com_rslt = bno055_get_accel_unit(&v_accel_unit_u8);
	if (v_accel_unit_u8 != ACCEL_UNIT_MSQ)
		com_rslt += bno055_set_accel_unit(ACCEL_UNIT_MSQ);
		if (com_rslt == SUCCESS) {
			com_rslt += bno055_read_accel_y(&v_reg_accel_y_s16);
			p_bno055->delay_msec(BNO055_ONE_U8X);
			if (com_rslt == SUCCESS) {
				/* Convert the raw accel y to m/s2*/
				v_data_f =
				(float)(v_reg_accel_y_s16/ACCEL_DIV_MSQ);
				*v_accel_y_f = v_data_f;
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the accel y raw data
 *	to millig output as float
 *
 *	@param v_accel_y_f : The accel y millig data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_accel_y_mg(
float *v_accel_y_f)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	s16 v_reg_accel_y_s16 = BNO055_ZERO_U8X;
	float data = BNO055_ZERO_U8X;
	u8 v_accel_unit_u8 = BNO055_ZERO_U8X;
	/* Read the current accel unit and set the
	unit as mg if the unit is in m/s2*/
	com_rslt = bno055_get_accel_unit(&v_accel_unit_u8);
	if (v_accel_unit_u8 != ACCEL_UNIT_MG)
		com_rslt += bno055_set_accel_unit(ACCEL_UNIT_MG);
		if (com_rslt == SUCCESS) {
			/* Read the accel raw z data*/
			com_rslt += bno055_read_accel_y(&v_reg_accel_y_s16);
			if (com_rslt == SUCCESS) {
				/* Convert the raw accel z to mg*/
				data = (float)(v_reg_accel_y_s16/ACCEL_DIV_MG);
				*v_accel_y_f = data;
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the accel z raw data
 *	to meterpersecseq output as float
 *
 *	@param v_accel_z_f : The accel z meterpersecseq data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_accel_z_msq(
float *v_accel_z_f)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	s16 v_reg_accel_z_s16 = BNO055_ZERO_U8X;
	float v_data_f = BNO055_ZERO_U8X;
	u8 v_accel_unit_u8 = BNO055_ZERO_U8X;
	/* Read the current accel unit and set the
	unit as m/s2 if the unit is in mg*/
	com_rslt = bno055_get_accel_unit(&v_accel_unit_u8);
	if (v_accel_unit_u8 != ACCEL_UNIT_MSQ)
		com_rslt += bno055_set_accel_unit(ACCEL_UNIT_MSQ);
		if (com_rslt == SUCCESS) {
			/* Read the accel raw z data*/
			com_rslt += bno055_read_accel_z(&v_reg_accel_z_s16);
			p_bno055->delay_msec(BNO055_ONE_U8X);
			if (com_rslt == SUCCESS) {
				/* Convert the raw accel z to m/s2*/
				v_data_f =
				(float)(v_reg_accel_z_s16/ACCEL_DIV_MSQ);
				*v_accel_z_f = v_data_f;
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the accel z raw data
 *	to millig output as float
 *
 *	@param v_accel_z_f : The accel z millig data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_accel_z_mg(
float *v_accel_z_f)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	s16 v_reg_accel_z_s16 = BNO055_ZERO_U8X;
	float v_data_f = BNO055_ZERO_U8X;
	u8 v_accel_unit_u8 = BNO055_ZERO_U8X;
	/* Read the current accel unit and set the
	unit as mg if the unit is in m/s2 */
	com_rslt = bno055_get_accel_unit(&v_accel_unit_u8);
	if (v_accel_unit_u8 != ACCEL_UNIT_MG)
		com_rslt += bno055_set_accel_unit(ACCEL_UNIT_MG);
		if (com_rslt == SUCCESS) {
			/* Read the accel raw z data*/
			com_rslt += bno055_read_accel_z(&v_reg_accel_z_s16);
			if (com_rslt == SUCCESS) {
				/* Convert the raw accel x to mg*/
				v_data_f =
				(float)(v_reg_accel_z_s16/ACCEL_DIV_MG);
				*v_accel_z_f = v_data_f;
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the accel xyz raw data
 *	to meterpersecseq output as float
 *
 *	@param accel_xyz : The meterpersecseq data of accel xyz
 *
 *	Parameter |    result
 *  --------- | -----------------
 *	 x        | meterpersecseq data of accel
 *	 y        | meterpersecseq data of accel
 *	 z        | meterpersecseq data of accel
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_accel_xyz_msq(
struct bno055_accel_float_t *accel_xyz)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	struct bno055_accel_t reg_accel_xyz = {
	BNO055_ZERO_U8X, BNO055_ZERO_U8X, BNO055_ZERO_U8X};
	u8 v_accel_unit_u8 = BNO055_ZERO_U8X;
	/* Read the current accel unit and set the
	unit as m/s2 if the unit is in mg*/
	com_rslt = bno055_get_accel_unit(&v_accel_unit_u8);
	if (v_accel_unit_u8 != ACCEL_UNIT_MSQ)
		com_rslt += bno055_set_accel_unit(ACCEL_UNIT_MSQ);
		if (com_rslt == SUCCESS) {
			/* Read the accel raw xyz data*/
			com_rslt += bno055_read_accel_xyz(&reg_accel_xyz);
			if (com_rslt == SUCCESS) {
				/* Convert the accel raw xyz to meterpersecseq*/
				accel_xyz->x =
				(float)(reg_accel_xyz.x/ACCEL_DIV_MSQ);
				accel_xyz->y =
				(float)(reg_accel_xyz.y/ACCEL_DIV_MSQ);
				accel_xyz->z =
				(float)(reg_accel_xyz.z/ACCEL_DIV_MSQ);
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the accel xyz raw data
 *	to millig output as float
 *
 *	@param accel_xyz : The millig data of accel xyz
 *
 *	Parameter |    result
 *  --------- | -----------------
 *	 x        | millig data of accel
 *	 y        | millig data of accel
 *	 z        | millig data of accel
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_accel_xyz_mg(
struct bno055_accel_float_t *accel_xyz)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	struct bno055_accel_t reg_accel_xyz = {
	BNO055_ZERO_U8X, BNO055_ZERO_U8X, BNO055_ZERO_U8X};
	u8 v_accel_unit_u8 = BNO055_ZERO_U8X;
	/* Read the current accel unit and set the
	unit as mg if the unit is in m/s2*/
	com_rslt = bno055_get_accel_unit(&v_accel_unit_u8);
	if (v_accel_unit_u8 != ACCEL_UNIT_MG)
		com_rslt += bno055_set_accel_unit(ACCEL_UNIT_MG);
		if (com_rslt == SUCCESS) {
			/* Read the accel raw y data*/
			com_rslt += bno055_read_accel_xyz(&reg_accel_xyz);
			if (com_rslt == SUCCESS) {
				/*Convert the accel raw xyz to millig */
				accel_xyz->x =
				(float)(reg_accel_xyz.x/ACCEL_DIV_MG);
				accel_xyz->y =
				(float)(reg_accel_xyz.y/ACCEL_DIV_MG);
				accel_xyz->z =
				(float)(reg_accel_xyz.z/ACCEL_DIV_MG);
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the mag x raw data
 *	to microTesla output as float
 *
 *	@param v_mag_x_f : The mag x microTesla data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_mag_x_uT(
float *v_mag_x_f)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	s16 v_reg_mag_x_s16 = BNO055_ZERO_U8X;
	float v_data_f = BNO055_ZERO_U8X;
	/* Read raw mag x data */
	com_rslt = bno055_read_mag_x(&v_reg_mag_x_s16);
	if (com_rslt == SUCCESS) {
		/* Convert the raw mag x to microTesla*/
		v_data_f = (float)(v_reg_mag_x_s16/MAG_DIV_UT);
		*v_mag_x_f = v_data_f;
	} else {
	com_rslt = ERROR;
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the mag y raw data
 *	to microTesla output as float
 *
 *	@param v_mag_y_f : The mag y microTesla data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_mag_y_uT(
float *v_mag_y_f)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	s16 v_reg_mag_y_s16 = BNO055_ZERO_U8X;
	float v_data_f = BNO055_ZERO_U8X;
	/* Read raw mag y data */
	com_rslt = bno055_read_mag_y(&v_reg_mag_y_s16);
	if (com_rslt == SUCCESS) {
		/* Convert the raw mag y to microTesla*/
		v_data_f = (float)(v_reg_mag_y_s16/MAG_DIV_UT);
		*v_mag_y_f = v_data_f;
	} else {
	com_rslt = ERROR;
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the mag z raw data
 *	to microTesla output as float
 *
 *	@param v_mag_z_f : The mag z microTesla data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_mag_z_uT(
float *v_mag_z_f)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	s16 v_reg_mag_z_s16 = BNO055_ZERO_U8X;
	float v_data_f = BNO055_ZERO_U8X;
	/* Read raw mag z data */
	com_rslt = bno055_read_mag_z(&v_reg_mag_z_s16);
	if (com_rslt == SUCCESS) {
		/* Convert the raw mag z to microTesla*/
		v_data_f = (float)(v_reg_mag_z_s16/MAG_DIV_UT);
		*v_mag_z_f = v_data_f;
	} else {
	com_rslt = ERROR;
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the mag yz raw data
 *	to microTesla output as float
 *
 *	@param mag_xyz_data : The microTesla data of mag xyz
 *
 *	Parameter |    result
 *  --------- | -----------------
 *	  x       | microTesla data of mag
 *	  y       | microTesla data of mag
 *	  z       | microTesla data of mag
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_mag_xyz_uT(
struct bno055_mag_float_t *mag_xyz_data)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	struct bno055_mag_t mag_xyz = {
	BNO055_ZERO_U8X, BNO055_ZERO_U8X, BNO055_ZERO_U8X};
	/* Read raw mag x data */
	com_rslt = bno055_read_mag_xyz(&mag_xyz);
	if (com_rslt == SUCCESS) {
		/* Convert mag raw xyz to microTesla*/
		mag_xyz_data->x = (float)(mag_xyz.x/MAG_DIV_UT);
		mag_xyz_data->y = (float)(mag_xyz.y/MAG_DIV_UT);
		mag_xyz_data->z = (float)(mag_xyz.z/MAG_DIV_UT);
	} else {
	com_rslt = ERROR;
	}

	return com_rslt;
}
/*!
 *	@brief This API is used to convert the gyro x raw data
 *	to dps output as float
 *
 *	@param v_gyro_x_f : The gyro x dps float data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_gyro_x_dps(
float *v_gyro_x_f)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	s16 v_reg_gyro_x_s16 = BNO055_ZERO_U8X;
	float v_data_f = BNO055_ZERO_U8X;
	u8 v_gyro_unit_u8 = BNO055_ZERO_U8X;
	/* Read the current gyro unit and set the
	unit as dps if the unit is in rps */
	com_rslt = bno055_get_gyro_unit(&v_gyro_unit_u8);
	if (v_gyro_unit_u8 != GYRO_UNIT_DPS)
		com_rslt += bno055_set_gyro_unit(GYRO_UNIT_DPS);
		if (com_rslt == SUCCESS) {
			/* Read gyro raw x data */
			com_rslt += bno055_read_gyro_x(&v_reg_gyro_x_s16);
			if (com_rslt == SUCCESS) {
				/* Convert the raw gyro x to dps*/
				v_data_f =
				(float)(v_reg_gyro_x_s16/GYRO_DIV_DPS);
				*v_gyro_x_f = v_data_f;
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the gyro x raw data
 *	to rps output as float
 *
 *	@param v_gyro_x_f : The gyro x dps float data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_gyro_x_rps(
float *v_gyro_x_f)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	s16 v_reg_gyro_x_s16 = BNO055_ZERO_U8X;
	float v_data_f = BNO055_ZERO_U8X;
	u8 v_gyro_unit_u8 = BNO055_ZERO_U8X;
	/* Read the current gyro unit and set the
	unit as rps if the unit is in dps */
	com_rslt = bno055_get_gyro_unit(&v_gyro_unit_u8);
	if (v_gyro_unit_u8 != GYRO_UNIT_RPS)
		com_rslt += bno055_set_gyro_unit(GYRO_UNIT_RPS);
		if (com_rslt == SUCCESS) {
			/* Read gyro raw x data */
			com_rslt += bno055_read_gyro_x(&v_reg_gyro_x_s16);
			if (com_rslt == SUCCESS) {
				/* Convert the raw gyro x to rps*/
				v_data_f =
				(float)(v_reg_gyro_x_s16/GYRO_DIV_RPS);
				*v_gyro_x_f = v_data_f;
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the gyro y raw data
 *	to dps output as float
 *
 *	@param v_gyro_y_f : The gyro y dps float data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_gyro_y_dps(
float *v_gyro_y_f)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	s16 v_reg_gyro_y_s16 = BNO055_ZERO_U8X;
	float v_data_f = BNO055_ZERO_U8X;
	u8 v_gyro_unit_u8 = BNO055_ZERO_U8X;
	/* Read the current gyro unit and set the
	unit as dps if the unit is in rps */
	com_rslt = bno055_get_gyro_unit(&v_gyro_unit_u8);
	if (v_gyro_unit_u8 != GYRO_UNIT_DPS)
		com_rslt += bno055_set_gyro_unit(GYRO_UNIT_DPS);
		if (com_rslt == SUCCESS) {
			/* Read gyro raw y data */
			com_rslt += bno055_read_gyro_y(&v_reg_gyro_y_s16);
			if (com_rslt == SUCCESS) {
				/* Convert the raw gyro x to dps*/
				v_data_f =
				(float)(v_reg_gyro_y_s16/GYRO_DIV_DPS);
				*v_gyro_y_f = v_data_f;
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the gyro y raw data
 *	to rps output as float
 *
 *	@param v_gyro_y_f : The gyro y dps float data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_gyro_y_rps(
float *v_gyro_y_f)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	s16 v_reg_gyro_y_s16 = BNO055_ZERO_U8X;
	float v_data_f = BNO055_ZERO_U8X;
	u8 v_gyro_unit_u8 = BNO055_ZERO_U8X;
	/* Read the current gyro unit and set the
	unit as rps if the unit is in dps */
	com_rslt = bno055_get_gyro_unit(&v_gyro_unit_u8);
	if (v_gyro_unit_u8 != GYRO_UNIT_RPS)
		com_rslt += bno055_set_gyro_unit(GYRO_UNIT_RPS);
		if (com_rslt == SUCCESS) {
			/* Read gyro raw y data */
			com_rslt += bno055_read_gyro_y(&v_reg_gyro_y_s16);
			if (com_rslt == SUCCESS) {
				/* Convert the raw gyro x to rps*/
				v_data_f =
				(float)(v_reg_gyro_y_s16/GYRO_DIV_RPS);
				*v_gyro_y_f = v_data_f;
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the gyro z raw data
 *	to dps output as float
 *
 *	@param v_gyro_z_f : The gyro z dps float data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_gyro_z_dps(
float *v_gyro_z_f)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	s16 v_reg_gyro_z_s16 = BNO055_ZERO_U8X;
	float v_data_f = BNO055_ZERO_U8X;
	u8 v_gyro_unit_u8 = BNO055_ZERO_U8X;
	/* Read the current gyro unit and set the
	unit as dps if the unit is in rps */
	com_rslt = bno055_get_gyro_unit(&v_gyro_unit_u8);
	if (v_gyro_unit_u8 != GYRO_UNIT_DPS)
		com_rslt += bno055_set_gyro_unit(GYRO_UNIT_DPS);
		if (com_rslt == SUCCESS) {
			/* Read gyro raw z data */
			com_rslt += bno055_read_gyro_z(&v_reg_gyro_z_s16);
			if (com_rslt == SUCCESS) {
				/* Convert the raw gyro x to dps*/
				v_data_f =
				(float)(v_reg_gyro_z_s16/GYRO_DIV_DPS);
				*v_gyro_z_f = v_data_f;
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the gyro z raw data
 *	to rps output as float
 *
 *	@param v_gyro_z_f : The gyro z rps float data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_gyro_z_rps(
float *v_gyro_z_f)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	s16 v_reg_gyro_z_s16 = BNO055_ZERO_U8X;
	float v_data_f = BNO055_ZERO_U8X;
	u8 v_gyro_unit_u8 = BNO055_ZERO_U8X;
	/* Read the current gyro unit and set the
	unit as rps if the unit is in dps */
	com_rslt = bno055_get_gyro_unit(&v_gyro_unit_u8);
	if (v_gyro_unit_u8 != GYRO_UNIT_RPS)
		com_rslt += bno055_set_gyro_unit(GYRO_UNIT_RPS);
		if (com_rslt == SUCCESS) {
			/* Read gyro raw x data */
			com_rslt += bno055_read_gyro_z(&v_reg_gyro_z_s16);
			if (com_rslt == SUCCESS) {
				/* Convert the raw gyro x to rps*/
				v_data_f =
				(float)(v_reg_gyro_z_s16/GYRO_DIV_RPS);
				*v_gyro_z_f = v_data_f;
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the gyro xyz raw data
 *	to dps output as float
 *
 *	@param gyro_xyz_data : The dps data of gyro xyz
 *
 *	Parameter |    result
 *  --------- | -----------------
 *	  x       | dps data of gyro
 *	  y       | dps data of gyro
 *	  z       | dps data of gyro
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_gyro_xyz_dps(
struct bno055_gyro_float_t *gyro_xyz_data)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	struct bno055_gyro_t gyro_xyz = {
	BNO055_ZERO_U8X, BNO055_ZERO_U8X, BNO055_ZERO_U8X};
	u8 v_gyro_unit_u8 = BNO055_ZERO_U8X;
	/* Read the current gyro unit and set the
	unit as dps if the unit is in rps */
	com_rslt = bno055_get_gyro_unit(&v_gyro_unit_u8);
	if (v_gyro_unit_u8 != GYRO_UNIT_DPS)
		com_rslt += bno055_set_gyro_unit(GYRO_UNIT_DPS);
		if (com_rslt == SUCCESS) {
			/* Read gyro raw xyz data */
			com_rslt += bno055_read_gyro_xyz(&gyro_xyz);
			if (com_rslt == SUCCESS) {
				/* Convert gyro raw xyz to dps*/
				gyro_xyz_data->x =
				(float)(gyro_xyz.x/GYRO_DIV_DPS);
				gyro_xyz_data->y =
				(float)(gyro_xyz.y/GYRO_DIV_DPS);
				gyro_xyz_data->z =
				(float)(gyro_xyz.z/GYRO_DIV_DPS);
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the gyro xyz raw data
 *	to rps output as float
 *
 *	@param gyro_xyz_data : The rps data of gyro xyz
 *
 *	Parameter |    result
 *  --------- | -----------------
 *	  x       | rps data of gyro
 *	  y       | rps data of gyro
 *	  z       | rps data of gyro
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_gyro_xyz_rps(
struct bno055_gyro_float_t *gyro_xyz_data)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	struct bno055_gyro_t gyro_xyz = {BNO055_ZERO_U8X,
	BNO055_ZERO_U8X, BNO055_ZERO_U8X};
	u8 v_gyro_unit_u8 = BNO055_ZERO_U8X;
	/* Read the current gyro unit and set the
	unit as rps if the unit is in dps */
	com_rslt = bno055_get_gyro_unit(&v_gyro_unit_u8);
	if (v_gyro_unit_u8 != GYRO_UNIT_RPS)
		com_rslt += bno055_set_gyro_unit(GYRO_UNIT_RPS);
		if (com_rslt == SUCCESS) {
			/* Read gyro raw xyz data */
			com_rslt += bno055_read_gyro_xyz(&gyro_xyz);
			if (com_rslt == SUCCESS) {
				/* Convert gyro raw xyz to rps*/
				gyro_xyz_data->x =
				(float)(gyro_xyz.x/GYRO_DIV_RPS);
				gyro_xyz_data->y =
				(float)(gyro_xyz.y/GYRO_DIV_RPS);
				gyro_xyz_data->z =
				(float)(gyro_xyz.z/GYRO_DIV_RPS);
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the Euler h raw data
 *	to degree output as float
 *
 *	@param v_euler_h_f : The float value of Euler h degree
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_euler_h_deg(
float *v_euler_h_f)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	s16 v_reg_euler_h_s16 = BNO055_ZERO_U8X;
	float v_data_f = BNO055_ZERO_U8X;
	u8 v_euler_unit_u8 = BNO055_ZERO_U8X;
	/* Read the current Euler unit and set the
	unit as degree if the unit is in radians */
	com_rslt = bno055_get_euler_unit(&v_euler_unit_u8);
	if (v_euler_unit_u8 != EULER_UNIT_DEG)
		com_rslt += bno055_set_euler_unit(EULER_UNIT_DEG);
		if (com_rslt ==  SUCCESS) {
			/* Read Euler raw h data*/
			com_rslt += bno055_read_euler_h(&v_reg_euler_h_s16);
			if (com_rslt == SUCCESS) {
				/* Convert raw Euler h data to degree*/
				v_data_f =
				(float)(v_reg_euler_h_s16/EULER_DIV_DEG);
				*v_euler_h_f = v_data_f;
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the Euler h raw data
 *	to radians output as float
 *
 *	@param v_euler_h_f : The float value of Euler h radians
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_euler_h_rad(
float *v_euler_h_f)
{
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	s16 v_reg_euler_h_s16 = BNO055_ZERO_U8X;
	float v_data_f = BNO055_ZERO_U8X;
	u8 v_euler_unit_u8 = BNO055_ZERO_U8X;
	com_rslt = bno055_get_euler_unit(&v_euler_unit_u8);
	if (v_euler_unit_u8 != EULER_UNIT_RAD)
		/* Read the current Euler unit and set the
		unit as radians if the unit is in degree */
		com_rslt += bno055_set_euler_unit(EULER_UNIT_RAD);
		if (com_rslt == SUCCESS) {
			/* Read Euler raw h data*/
			com_rslt += bno055_read_euler_h(&v_reg_euler_h_s16);
			if (com_rslt == SUCCESS) {
				/* Convert raw Euler h data to degree*/
				v_data_f =
				(float)(v_reg_euler_h_s16/EULER_DIV_RAD);
				*v_euler_h_f = v_data_f;
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the Euler r raw data
 *	to degree output as float
 *
 *	@param v_euler_r_f : The float value of Euler r degree
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_euler_r_deg(
float *v_euler_r_f)
{
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	s16 reg_euler_r = BNO055_ZERO_U8X;
	float v_data_f = BNO055_ZERO_U8X;
	u8 v_euler_unit_u8 = BNO055_ZERO_U8X;
	/* Read the current Euler unit and set the
	unit as degree if the unit is in radians */
	com_rslt = bno055_get_euler_unit(&v_euler_unit_u8);
	if (v_euler_unit_u8 != EULER_UNIT_DEG)
		com_rslt += bno055_set_euler_unit(EULER_UNIT_DEG);
		if (com_rslt ==  SUCCESS) {
			/* Read Euler raw r data*/
			com_rslt += bno055_read_euler_r(&reg_euler_r);
			if (com_rslt == SUCCESS) {
				/* Convert raw Euler r data to degree*/
				v_data_f = (float)(reg_euler_r/EULER_DIV_DEG);
				*v_euler_r_f = v_data_f;
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the Euler r raw data
 *	to radians output as float
 *
 *	@param v_euler_r_f : The float value of Euler r radians
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_euler_r_rad(
float *v_euler_r_f)
{
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	s16 reg_v_euler_r_f = BNO055_ZERO_U8X;
	float v_data_f = BNO055_ZERO_U8X;
	u8 v_euler_unit_u8 = BNO055_ZERO_U8X;
	/* Read the current Euler unit and set the
	unit as radians if the unit is in degree */
	com_rslt = bno055_get_euler_unit(&v_euler_unit_u8);
	if (v_euler_unit_u8 != EULER_UNIT_RAD)
		com_rslt += bno055_set_euler_unit(EULER_UNIT_RAD);
		if (com_rslt == SUCCESS) {
			/* Read Euler raw r data*/
			com_rslt += bno055_read_euler_r(&reg_v_euler_r_f);
			if (com_rslt == SUCCESS) {
				/* Convert raw Euler r data to radians*/
				v_data_f =
				(float)(reg_v_euler_r_f/EULER_DIV_RAD);
				*v_euler_r_f = v_data_f;
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the Euler p raw data
 *	to degree output as float
 *
 *	@param v_euler_p_f : The float value of Euler p degree
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_euler_p_deg(
float *v_euler_p_f)
{
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	s16 reg_v_euler_p_f = BNO055_ZERO_U8X;
	float v_data_f = BNO055_ZERO_U8X;
	u8 v_euler_unit_u8 = BNO055_ZERO_U8X;
	/* Read the current Euler unit and set the
	unit as degree if the unit is in radians */
	com_rslt = bno055_get_euler_unit(&v_euler_unit_u8);
	if (v_euler_unit_u8 != EULER_UNIT_DEG)
		com_rslt += bno055_set_euler_unit(EULER_UNIT_DEG);
		if (com_rslt ==  SUCCESS) {
			/* Read Euler raw p data*/
			com_rslt += bno055_read_euler_p(&reg_v_euler_p_f);
			if (com_rslt == SUCCESS) {
				/* Convert raw Euler p data to degree*/
				v_data_f =
				(float)(reg_v_euler_p_f/EULER_DIV_DEG);
				*v_euler_p_f = v_data_f;
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the Euler p raw data
 *	to radians output as float
 *
 *	@param v_euler_p_f : The float value of Euler p radians
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_euler_p_rad(
float *v_euler_p_f)
{
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	s16 reg_v_euler_p_f = BNO055_ZERO_U8X;
	float v_data_f = BNO055_ZERO_U8X;
	u8 v_euler_unit_u8 = BNO055_ZERO_U8X;
	/* Read the current Euler unit and set the
	unit as radians if the unit is in degree */
	com_rslt = bno055_get_euler_unit(&v_euler_unit_u8);
	if (v_euler_unit_u8 != EULER_UNIT_RAD)
		com_rslt += bno055_set_euler_unit(EULER_UNIT_RAD);
		if (com_rslt == SUCCESS) {
			/* Read Euler raw r data*/
			com_rslt += bno055_read_euler_p(&reg_v_euler_p_f);
			if (com_rslt == SUCCESS) {
				/* Convert raw Euler r data to radians*/
				v_data_f =
				(float)(reg_v_euler_p_f/EULER_DIV_RAD);
				*v_euler_p_f = v_data_f;
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the Euler hrp raw data
 *	to degree output as float
 *
 *	@param euler_hpr : The degree data of Euler hrp
 *
 *	Parameter |    result
 *  --------- | -----------------
 *	  h       | degree data of Euler
 *	  r       | degree data of Euler
 *	  p       | degree data of Euler
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_euler_hpr_deg(
struct bno055_euler_float_t *euler_hpr)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	struct bno055_euler_t reg_euler = {BNO055_ZERO_U8X,
	BNO055_ZERO_U8X, BNO055_ZERO_U8X};
	u8 v_euler_unit_u8 = BNO055_ZERO_U8X;
	/* Read the current Euler unit and set the
	unit as degree if the unit is in radians */
	com_rslt = bno055_get_euler_unit(&v_euler_unit_u8);
	if (v_euler_unit_u8 != EULER_UNIT_DEG)
		com_rslt += bno055_set_euler_unit(EULER_UNIT_DEG);
		if (com_rslt == SUCCESS) {
			/* Read Euler raw hrp data*/
			com_rslt += bno055_read_euler_hrp(&reg_euler);
			if (com_rslt == SUCCESS) {
				/* Convert raw Euler hrp to degree*/
				euler_hpr->h =
				(float)(reg_euler.h/EULER_DIV_DEG);
				euler_hpr->p =
				(float)(reg_euler.p/EULER_DIV_DEG);
				euler_hpr->r =
				(float)(reg_euler.r/EULER_DIV_DEG);
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the Euler xyz raw data
 *	to radians output as float
 *
 *	@param euler_hpr : The radians data of Euler hrp
 *
 *	Parameter |    result
 *  --------- | -----------------
 *	  h       | radians data of Euler
 *	  r       | radians data of Euler
 *	  p       | radians data of Euler
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_euler_hpr_rad(
struct bno055_euler_float_t *euler_hpr)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	struct bno055_euler_t reg_euler = {BNO055_ZERO_U8X,
	BNO055_ZERO_U8X, BNO055_ZERO_U8X};
	u8 v_euler_unit_u8 = BNO055_ZERO_U8X;
	/* Read the current Euler unit and set the
	unit as radians if the unit is in degree */
	com_rslt = bno055_get_euler_unit(&v_euler_unit_u8);
	if (v_euler_unit_u8 != EULER_UNIT_RAD)
		com_rslt += bno055_set_euler_unit(EULER_UNIT_RAD);
		if (com_rslt == SUCCESS) {
			/* Read Euler raw hrp data*/
			com_rslt += bno055_read_euler_hrp(&reg_euler);
			if (com_rslt == SUCCESS) {
				/* Convert raw hrp to radians */
				euler_hpr->h =
				(float)(reg_euler.h/EULER_DIV_RAD);
				euler_hpr->p =
				(float)(reg_euler.p/EULER_DIV_RAD);
				euler_hpr->r =
				(float)(reg_euler.r/EULER_DIV_RAD);
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the linear
 *	accel x raw data to meterpersecseq output as float
 *
 *	@param v_linear_accel_x_f : The float value of linear accel x meterpersecseq
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_linear_accel_x_msq(
float *v_linear_accel_x_f)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	s16 v_reg_linear_accel_x_s16 = BNO055_ZERO_U8X;
	float v_data_f = BNO055_ZERO_U8X;
	/* Read the raw x of linear accel */
	com_rslt = bno055_read_linear_accel_x(&v_reg_linear_accel_x_s16);
	if (com_rslt == SUCCESS) {
		/* Convert the raw linear accel x to m/s2*/
		v_data_f =
		(float)(v_reg_linear_accel_x_s16/LINEAR_ACCEL_DIV_MSQ);
		*v_linear_accel_x_f = v_data_f;
	} else {
		com_rslt = ERROR;
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the linear
 *	accel y raw data to meterpersecseq output as float
 *
 *	@param v_linear_accel_y_f : The float value of linear accel y meterpersecseq
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_linear_accel_y_msq(
float *v_linear_accel_y_f)
{
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	s16 reg_linear_accel_y = BNO055_ZERO_U8X;
	float v_data_f = BNO055_ZERO_U8X;
	/* Read the raw y of linear accel */
	com_rslt = bno055_read_linear_accel_y(&reg_linear_accel_y);
	if (com_rslt == SUCCESS) {
		/* Convert the raw linear accel x to m/s2*/
		v_data_f = (float)
		(reg_linear_accel_y/LINEAR_ACCEL_DIV_MSQ);
		*v_linear_accel_y_f = v_data_f;
	} else {
		com_rslt = ERROR;
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the linear
 *	accel z raw data to meterpersecseq output as float
 *
 *	@param v_linear_accel_z_f : The float value of linear accel z meterpersecseq
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_linear_accel_z_msq(
float *v_linear_accel_z_f)
{
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	s16 reg_linear_accel_z = BNO055_ZERO_U8X;
	float v_data_f = BNO055_ZERO_U8X;
	/* Read the raw x of linear accel */
	com_rslt = bno055_read_linear_accel_z(&reg_linear_accel_z);
	if (com_rslt == SUCCESS) {
		/* Convert the raw linear accel z to m/s2*/
		v_data_f = (float)
		(reg_linear_accel_z/LINEAR_ACCEL_DIV_MSQ);
		*v_linear_accel_z_f = v_data_f;
	} else {
		com_rslt = ERROR;
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the linear accel xyz raw data
 *	to meterpersecseq output as float
 *
 *	@param linear_accel_xyz : The meterpersecseq data of linear accel xyz
 *
 *	Parameter |    result
 *  --------- | -----------------
 *	  x       | meterpersecseq data of linear accel
 *	  y       | meterpersecseq data of linear accel
 *	  z       | meterpersecseq data of linear accel
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_linear_accel_xyz_msq(
struct bno055_linear_accel_float_t *linear_accel_xyz)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	struct bno055_linear_accel_t reg_linear_accel = {
	BNO055_ZERO_U8X, BNO055_ZERO_U8X, BNO055_ZERO_U8X};
	/* Read the raw x of linear accel */
	com_rslt = bno055_read_linear_accel_xyz(&reg_linear_accel);
	if (com_rslt == SUCCESS) {
		linear_accel_xyz->x =
		(float)(reg_linear_accel.x/LINEAR_ACCEL_DIV_MSQ);
		linear_accel_xyz->y =
		(float)(reg_linear_accel.y/LINEAR_ACCEL_DIV_MSQ);
		linear_accel_xyz->z =
		(float)(reg_linear_accel.z/LINEAR_ACCEL_DIV_MSQ);
	} else {
		com_rslt = ERROR;
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the gravity
 *	x raw data to meterpersecseq output as float
 *
 *	@param v_gravity_x_f : The float value of gravity x meterpersecseq
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_gravity_float_x_msq(
float *v_gravity_x_f)
{
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	s16 v_reg_gravity_x_s16 = BNO055_ZERO_U8X;
	float v_data_f = BNO055_ZERO_U8X;
	/* Read raw gravity of x*/
	com_rslt = bno055_read_gravity_x(&v_reg_gravity_x_s16);
	if (com_rslt == SUCCESS) {
		/* Convert the raw gravity x to m/s2*/
		v_data_f = (float)(v_reg_gravity_x_s16/GRAVITY_DIV_MSQ);
		*v_gravity_x_f = v_data_f;
	} else {
		com_rslt = ERROR;
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the gravity
 *	y raw data to meterpersecseq output as float
 *
 *	@param v_gravity_y_f : The float value of gravity y meterpersecseq
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_gravity_float_y_msq(
float *v_gravity_y_f)
{
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	s16 v_reg_gravity_y_s16 = BNO055_ZERO_U8X;
	float v_data_f = BNO055_ZERO_U8X;
	/* Read raw gravity of y*/
	com_rslt = bno055_read_gravity_y(&v_reg_gravity_y_s16);
	if (com_rslt == SUCCESS) {
		/* Convert the raw gravity y to m/s2*/
		v_data_f = (float)(v_reg_gravity_y_s16/GRAVITY_DIV_MSQ);
		*v_gravity_y_f = v_data_f;
	} else {
		com_rslt = ERROR;
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the gravity
 *	z raw data to meterpersecseq output as float
 *
 *	@param v_gravity_z_f : The float value of gravity z meterpersecseq
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_gravity_float_z_msq(
float *v_gravity_z_f)
{
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	s16 v_reg_gravity_z_s16 = BNO055_ZERO_U8X;
	float v_data_f = BNO055_ZERO_U8X;
	/* Read raw gravity of z */
	com_rslt = bno055_read_gravity_z(&v_reg_gravity_z_s16);
	if (com_rslt == SUCCESS) {
		/* Convert the raw gravity z to m/s2*/
		v_data_f = (float)(v_reg_gravity_z_s16/GRAVITY_DIV_MSQ);
		*v_gravity_z_f = v_data_f;
	} else {
		com_rslt = ERROR;
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the gravity xyz raw data
 *	to meterpersecseq output as float
 *
 *	@param gravity_xyz : The meterpersecseq data of gravity xyz
 *
 *	Parameter |    result
 *  --------- | -----------------
 *	  x       | meterpersecseq data of gravity
 *	  y       | meterpersecseq data of gravity
 *	  z       | meterpersecseq data of gravity
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_gravity_xyz_msq(
struct bno055_gravity_float_t *gravity_xyz)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	struct bno055_gravity_t reg_gravity_xyz = {
	BNO055_ZERO_U8X, BNO055_ZERO_U8X, BNO055_ZERO_U8X};
	/* Read raw gravity of xyz */
	com_rslt = bno055_read_gravity_xyz(&reg_gravity_xyz);
	if (com_rslt == SUCCESS) {
		/* Convert raw gravity xyz to meterpersecseq */
		gravity_xyz->x =
		(float)(reg_gravity_xyz.x/GRAVITY_DIV_MSQ);
		gravity_xyz->y =
		(float)(reg_gravity_xyz.y/GRAVITY_DIV_MSQ);
		gravity_xyz->z =
		(float)(reg_gravity_xyz.z/GRAVITY_DIV_MSQ);
	} else {
		com_rslt = ERROR;
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the temperature
 *	data to Fahrenheit output as float
 *
 *	@param v_temp_f : The float value of temperature Fahrenheit
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_temp_fahrenheit(
float *v_temp_f)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	s8 v_reg_temp_s8 = BNO055_ZERO_U8X;
	float v_data_f = BNO055_ZERO_U8X;
	u8 v_temp_unit_u8 = BNO055_ZERO_U8X;
	/* Read the current temperature unit and set the
	unit as Fahrenheit if the unit is in Celsius */
	com_rslt = bno055_get_temp_unit(&v_temp_unit_u8);
	if (v_temp_unit_u8 != TEMP_UNIT_FAHRENHEIT)
		com_rslt += bno055_set_temp_unit(TEMP_UNIT_FAHRENHEIT);
		if (com_rslt == SUCCESS) {
			/* Read the raw temperature data */
			com_rslt += bno055_read_temp_data(&v_reg_temp_s8);
			if (com_rslt == SUCCESS) {
				/* Convert raw temperature data to Fahrenheit*/
				v_data_f = (float)
				(v_reg_temp_s8/TEMP_DIV_FAHRENHEIT);
				*v_temp_f = v_data_f;
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the temperature
 *	data to Celsius output as float
 *
 *	@param v_temp_f : The float value of temperature Celsius
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_temp_celsius(
float *v_temp_f)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	s8 v_reg_temp_s8 = BNO055_ZERO_U8X;
	float v_data_f = BNO055_ZERO_U8X;
	u8 v_temp_unit_u8 = BNO055_ZERO_U8X;
	/* Read the current temperature unit and set the
	unit as Fahrenheit if the unit is in Celsius */
	com_rslt = bno055_get_temp_unit(&v_temp_unit_u8);
	if (v_temp_unit_u8 != TEMP_UNIT_CELSIUS)
		com_rslt += bno055_set_temp_unit(TEMP_UNIT_CELSIUS);
		if (com_rslt ==  SUCCESS) {
			/* Read the raw temperature data */
			com_rslt += bno055_read_temp_data(&v_reg_temp_s8);
			if (com_rslt == SUCCESS) {
				/* Convert raw temperature data to Fahrenheit*/
				v_data_f =
				(float)(v_reg_temp_s8/TEMP_DIV_CELSIUS);
				*v_temp_f = v_data_f;
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	return com_rslt;
}
#endif
#ifdef	BNO055_DOUBLE_ENABLE
/*!
 *	@brief This API is used to convert the accel x raw data
 *	to meterpersecseq output as double
 *
 *	@param v_accel_x_d : The accel x meterpersecseq data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_accel_x_msq(
double *v_accel_x_d)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	s16 v_reg_accel_x_s16 = BNO055_ZERO_U8X;
	double v_data_f = BNO055_ZERO_U8X;
	u8 v_accel_unit_u8 = BNO055_ZERO_U8X;
	/* Read the current accel unit and set the
	unit as m/s2 if the unit is in mg*/
	com_rslt = bno055_get_accel_unit(&v_accel_unit_u8);
	if (v_accel_unit_u8 != ACCEL_UNIT_MSQ)
		com_rslt += bno055_set_accel_unit(ACCEL_UNIT_MSQ);
		if (com_rslt == SUCCESS) {
			/* Read the accel raw y data*/
			com_rslt += bno055_read_accel_x(&v_reg_accel_x_s16);
			if (com_rslt == SUCCESS) {
				/* Convert the raw x to m/s2 */
				v_data_f =
				(double)(v_reg_accel_x_s16/ACCEL_DIV_MSQ);
				*v_accel_x_d = v_data_f;
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the accel x raw data
 *	to millig output as double
 *
 *	@param v_accel_x_d : The accel x millig data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_accel_x_mg(
double *v_accel_x_d)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	s16 v_reg_accel_x_s16 = BNO055_ZERO_U8X;
	double v_data_f = BNO055_ZERO_U8X;
	u8 v_accel_unit_u8 = BNO055_ZERO_U8X;
	/* Read the current accel unit and set the
	unit as mg if the unit is in m/s2*/
	com_rslt = bno055_get_accel_unit(&v_accel_unit_u8);
	if (v_accel_unit_u8 != ACCEL_UNIT_MG)
		com_rslt += bno055_set_accel_unit(ACCEL_UNIT_MG);
		if (com_rslt == SUCCESS) {
			/* Read the accel raw y data*/
			com_rslt += bno055_read_accel_x(&v_reg_accel_x_s16);
			if (com_rslt == SUCCESS) {
				/* Convert the raw x to mg */
				v_data_f =
				(double)(v_reg_accel_x_s16/ACCEL_DIV_MG);
				*v_accel_x_d = v_data_f;
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the accel y raw data
 *	to meterpersecseq output as double
 *
 *	@param v_accel_y_d : The accel y meterpersecseq data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_accel_y_msq(
double *v_accel_y_d)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	s16 v_reg_accel_y_s16 = BNO055_ZERO_U8X;
	double v_data_f = BNO055_ZERO_U8X;
	u8 v_accel_unit_u8 = BNO055_ZERO_U8X;
	/* Read the current accel unit and set the
	unit as m/s2 if the unit is in mg*/
	com_rslt = bno055_get_accel_unit(&v_accel_unit_u8);
	if (v_accel_unit_u8 != ACCEL_UNIT_MSQ)
		com_rslt += bno055_set_accel_unit(ACCEL_UNIT_MSQ);
		if (com_rslt == SUCCESS) {
			/* Read the accel raw y data*/
			com_rslt += bno055_read_accel_y(&v_reg_accel_y_s16);
			if (com_rslt == SUCCESS) {
				/* Convert the raw x to m/s2 */
				v_data_f =
				(double)(v_reg_accel_y_s16/ACCEL_DIV_MSQ);
				*v_accel_y_d = v_data_f;
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the accel y raw data
 *	to millig output as double
 *
 *	@param v_accel_y_d : The accel y millig data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_accel_y_mg(
double *v_accel_y_d)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	s16 v_reg_accel_y_s16 = BNO055_ZERO_U8X;
	double v_data_d = BNO055_ZERO_U8X;
	u8 v_accel_unit_u8 = BNO055_ZERO_U8X;
	/* Read the current accel unit and set the
	unit as mg if the unit is in m/s2*/
	com_rslt = bno055_get_accel_unit(&v_accel_unit_u8);
	if (v_accel_unit_u8 != ACCEL_UNIT_MG)
		com_rslt += bno055_set_accel_unit(ACCEL_UNIT_MG);
		if (com_rslt == SUCCESS) {
			/* Read the accel raw y data*/
			com_rslt += bno055_read_accel_y(&v_reg_accel_y_s16);
			if (com_rslt == SUCCESS) {
				/* Convert the raw y to mg */
				v_data_d =
				(double)(v_reg_accel_y_s16/ACCEL_DIV_MG);
				*v_accel_y_d = v_data_d;
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the accel z raw data
 *	to meterpersecseq output as double
 *
 *	@param v_accel_z_d : The accel z meterpersecseq data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_accel_z_msq(
double *v_accel_z_d)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	s16 v_reg_accel_z_s16 = BNO055_ZERO_U8X;
	double v_data_d = BNO055_ZERO_U8X;
	u8 v_accel_unit_u8 = BNO055_ZERO_U8X;
	/* Read the current accel unit and set the
	unit as m/s2 if the unit is in mg*/
	com_rslt = bno055_get_accel_unit(&v_accel_unit_u8);
	if (v_accel_unit_u8 != ACCEL_UNIT_MSQ)
		com_rslt += bno055_set_accel_unit(ACCEL_UNIT_MSQ);
		if (com_rslt == SUCCESS) {
			/* Read the accel raw z data*/
			com_rslt += bno055_read_accel_z(&v_reg_accel_z_s16);
			if (com_rslt == SUCCESS) {
				/* Convert the raw z to m/s2 */
				v_data_d =
				(double)(v_reg_accel_z_s16/ACCEL_DIV_MSQ);
				*v_accel_z_d = v_data_d;
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the accel z raw data
 *	to millig output as double
 *
 *	@param v_accel_z_d : The accel z millig data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_accel_z_mg(
double *v_accel_z_d)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	s16 v_reg_accel_z_s16 = BNO055_ZERO_U8X;
	double v_data_d = BNO055_ZERO_U8X;
	u8 v_accel_unit_u8 = BNO055_ZERO_U8X;
	/* Read the current accel unit and set the
	unit as mg if the unit is in m/s2*/
	com_rslt = bno055_get_accel_unit(&v_accel_unit_u8);
	if (v_accel_unit_u8 != ACCEL_UNIT_MG)
		com_rslt += bno055_set_accel_unit(ACCEL_UNIT_MG);
		if (com_rslt == SUCCESS) {
			/* Read the accel raw z data*/
			com_rslt += bno055_read_accel_z(&v_reg_accel_z_s16);
			if (com_rslt == SUCCESS) {
				/* Convert the raw z to mg */
				v_data_d =
				(double)(v_reg_accel_z_s16/ACCEL_DIV_MG);
				*v_accel_z_d = v_data_d;
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the accel xyz raw data
 *	to meterpersecseq output as double
 *
 *	@param accel_xyz : The meterpersecseq data of accel xyz
 *
 *	Parameter |    result
 *  --------- | -----------------
 *	 x        | meterpersecseq data of accel
 *	 y        | meterpersecseq data of accel
 *	 z        | meterpersecseq data of accel
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_accel_xyz_msq(
struct bno055_accel_double_t *accel_xyz)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	struct bno055_accel_t reg_accel_xyz = {
	BNO055_ZERO_U8X, BNO055_ZERO_U8X, BNO055_ZERO_U8X};
	u8 v_accel_unit_u8 = BNO055_ZERO_U8X;
	/* Read the current accel unit and set the
	unit as m/s2 if the unit is in mg*/
	com_rslt = bno055_get_accel_unit(&v_accel_unit_u8);
	if (v_accel_unit_u8 != ACCEL_UNIT_MSQ)
		com_rslt += bno055_set_accel_unit(ACCEL_UNIT_MSQ);
		if (com_rslt == SUCCESS) {
			/* Read the accel raw xyz data*/
			com_rslt += bno055_read_accel_xyz(&reg_accel_xyz);
			if (com_rslt == SUCCESS) {
				/* Convert raw xyz to m/s2*/
				accel_xyz->x =
				(double)(reg_accel_xyz.x/ACCEL_DIV_MSQ);
				accel_xyz->y =
				(double)(reg_accel_xyz.y/ACCEL_DIV_MSQ);
				accel_xyz->z =
				(double)(reg_accel_xyz.z/ACCEL_DIV_MSQ);
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the accel xyz raw data
 *	to millig output as double
 *
 *	@param accel_xyz : The millig data of accel xyz
 *
 *	Parameter |    result
 *  --------- | -----------------
 *	 x        | millig data of accel
 *	 y        | millig data of accel
 *	 z        | millig data of accel
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_accel_xyz_mg(
struct bno055_accel_double_t *accel_xyz)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	struct bno055_accel_t reg_accel_xyz = {
	BNO055_ZERO_U8X, BNO055_ZERO_U8X, BNO055_ZERO_U8X};
	u8 v_accel_unit_u8 = BNO055_ZERO_U8X;
	/* Read the current accel unit and set the
	unit as m/s2 if the unit is in mg*/
	com_rslt = bno055_get_accel_unit(&v_accel_unit_u8);
	if (v_accel_unit_u8 != ACCEL_UNIT_MG)
		com_rslt += bno055_set_accel_unit(ACCEL_UNIT_MG);
		if (com_rslt == SUCCESS) {
			/* Read the accel raw xyz data*/
			com_rslt += bno055_read_accel_xyz(&reg_accel_xyz);
			if (com_rslt == SUCCESS) {
				/* Convert raw xyz to mg*/
				accel_xyz->x =
				(double)(reg_accel_xyz.x/ACCEL_DIV_MG);
				accel_xyz->y =
				(double)(reg_accel_xyz.y/ACCEL_DIV_MG);
				accel_xyz->z =
				(double)(reg_accel_xyz.z/ACCEL_DIV_MG);
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the mag x raw data
 *	to microTesla output as double
 *
 *	@param v_mag_x_d : The mag x microTesla data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_mag_x_uT(
double *v_mag_x_d)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	s16 v_reg_v_mag_x_s16 = BNO055_ZERO_U8X;
	double v_data_d = BNO055_ZERO_U8X;
	/* Read raw mag x data */
	com_rslt = bno055_read_mag_x(&v_reg_v_mag_x_s16);
	if (com_rslt == SUCCESS) {
		/* Convert raw mag x to microTesla */
		v_data_d = (double)(v_reg_v_mag_x_s16/MAG_DIV_UT);
		*v_mag_x_d = v_data_d;
	} else {
	com_rslt = ERROR;
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the mag y raw data
 *	to microTesla output as double
 *
 *	@param v_mag_y_d : The mag y microTesla data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_mag_y_uT(
double *v_mag_y_d)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	s16 v_reg_mag_y_s16 = BNO055_ZERO_U8X;
	double v_data_d = BNO055_ZERO_U8X;
	/* Read raw mag y data */
	com_rslt = bno055_read_mag_y(&v_reg_mag_y_s16);
	if (com_rslt == SUCCESS) {
		/* Convert raw mag y to microTesla */
		v_data_d = (double)(v_reg_mag_y_s16/MAG_DIV_UT);
		*v_mag_y_d = v_data_d;
	} else {
	com_rslt = ERROR;
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the mag z raw data
 *	to microTesla output as double
 *
 *	@param v_mag_z_d : The mag z microTesla data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_mag_z_uT(
double *v_mag_z_d)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	s16 v_reg_mag_z_s16 = BNO055_ZERO_U8X;
	double v_data_d = BNO055_ZERO_U8X;
	/* Read raw mag x */
	com_rslt = bno055_read_mag_z(&v_reg_mag_z_s16);
	if (com_rslt == SUCCESS) {
		/* Convert raw mag x to microTesla */
		v_data_d = (double)(v_reg_mag_z_s16/MAG_DIV_UT);
		*v_mag_z_d = v_data_d;
	} else {
	com_rslt = ERROR;
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the mag yz raw data
 *	to microTesla output as double
 *
 *	@param mag_xyz : The microTesla data of mag xyz
 *
 *	Parameter |    result
 *  --------- | -----------------
 *	 x        | microTesla data of mag
 *	 y        | microTesla data of mag
 *	 z        | microTesla data of mag
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_mag_xyz_uT(
struct bno055_mag_double_t *mag_xyz)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	struct bno055_mag_t reg_mag_xyz = {
	BNO055_ZERO_U8X, BNO055_ZERO_U8X, BNO055_ZERO_U8X};
	/* Read raw mag xyz data */
	com_rslt = bno055_read_mag_xyz(&reg_mag_xyz);
	if (com_rslt == SUCCESS) {
		/* Convert raw mag xyz to microTesla*/
		mag_xyz->x =
		(double)(reg_mag_xyz.x/MAG_DIV_UT);
		mag_xyz->y =
		(double)(reg_mag_xyz.y/MAG_DIV_UT);
		mag_xyz->z =
		(double)(reg_mag_xyz.z/MAG_DIV_UT);
	} else {
	com_rslt = ERROR;
	}

	return com_rslt;
}
/*!
 *	@brief This API is used to convert the gyro x raw data
 *	to dps output as double
 *
 *	@param v_gyro_x_d : The gyro x dps double data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_gyro_x_dps(
double *v_gyro_x_d)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	s16 v_reg_gyro_x_s16 = BNO055_ZERO_U8X;
	double v_data_d = BNO055_ZERO_U8X;
	u8 v_gyro_unit_u8 = BNO055_ZERO_U8X;
	/* Read the current gyro unit and set the
	unit as dps if the unit is in rps */
	com_rslt = bno055_get_gyro_unit(&v_gyro_unit_u8);
	if (v_gyro_unit_u8 != GYRO_UNIT_DPS)
		com_rslt += bno055_set_gyro_unit(GYRO_UNIT_DPS);
		if (com_rslt == SUCCESS) {
			/* Read gyro raw x data */
			com_rslt += bno055_read_gyro_x(&v_reg_gyro_x_s16);
			if (com_rslt == SUCCESS) {
				/* Convert raw gyro x to dps */
				v_data_d =
				(double)(v_reg_gyro_x_s16/GYRO_DIV_DPS);
				*v_gyro_x_d = v_data_d;
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the gyro x raw data
 *	to rps output as double
 *
 *	@param v_gyro_x_d : The gyro x dps double data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_gyro_x_rps(
double *v_gyro_x_d)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	s16 v_reg_gyro_x_s16 = BNO055_ZERO_U8X;
	double v_data_d = BNO055_ZERO_U8X;
	u8 v_gyro_unit_u8 = BNO055_ZERO_U8X;
	/* Read the current gyro unit and set the
	unit as rps if the unit is in dps */
	com_rslt = bno055_get_gyro_unit(&v_gyro_unit_u8);
	if (v_gyro_unit_u8 != GYRO_UNIT_RPS)
		com_rslt += bno055_set_gyro_unit(GYRO_UNIT_RPS);
		if (com_rslt == SUCCESS) {
			/* Read gyro raw x data */
			com_rslt += bno055_read_gyro_x(&v_reg_gyro_x_s16);
			if (com_rslt == SUCCESS) {
				/* Convert raw gyro x to rps */
				v_data_d =
				(double)(v_reg_gyro_x_s16/GYRO_DIV_RPS);
				*v_gyro_x_d = v_data_d;
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the gyro y raw data
 *	to dps output as double
 *
 *	@param v_gyro_y_d : The gyro y dps double data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_gyro_y_dps(
double *v_gyro_y_d)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	s16 v_reg_gyro_y_s16 = BNO055_ZERO_U8X;
	double v_data_d = BNO055_ZERO_U8X;
	u8 v_gyro_unit_u8 = BNO055_ZERO_U8X;
	/* Read the current gyro unit and set the
	unit as dps if the unit is in rps */
	com_rslt = bno055_get_gyro_unit(&v_gyro_unit_u8);
	if (v_gyro_unit_u8 != GYRO_UNIT_DPS)
		com_rslt += bno055_set_gyro_unit(GYRO_UNIT_DPS);
		if (com_rslt == SUCCESS) {
			/* Read gyro raw y data */
			com_rslt += bno055_read_gyro_y(&v_reg_gyro_y_s16);
			if (com_rslt == SUCCESS) {
				/* Convert raw gyro y to dps */
				v_data_d =
				(double)(v_reg_gyro_y_s16/GYRO_DIV_DPS);
				*v_gyro_y_d = v_data_d;
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the gyro y raw data
 *	to rps output as double
 *
 *	@param v_gyro_y_d : The gyro y dps double data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_gyro_y_rps(
double *v_gyro_y_d)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	s16 v_reg_gyro_y_s16 = BNO055_ZERO_U8X;
	double v_data_d = BNO055_ZERO_U8X;
	u8 v_gyro_unit_u8 = BNO055_ZERO_U8X;
	/* Read the current gyro unit and set the
	unit as rps if the unit is in dps */
	com_rslt = bno055_get_gyro_unit(&v_gyro_unit_u8);
	if (v_gyro_unit_u8 != GYRO_UNIT_RPS)
		com_rslt += bno055_set_gyro_unit(GYRO_UNIT_RPS);
		if (com_rslt == SUCCESS) {
			/* Read gyro raw y data */
			com_rslt += bno055_read_gyro_y(&v_reg_gyro_y_s16);
			if (com_rslt == SUCCESS) {
				/* Convert raw gyro y to rps */
				v_data_d =
				(double)(v_reg_gyro_y_s16/GYRO_DIV_RPS);
				*v_gyro_y_d = v_data_d;
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the gyro z raw data
 *	to dps output as double
 *
 *	@param v_gyro_z_d : The gyro z dps double data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_gyro_z_dps(
double *v_gyro_z_d)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	s16 v_reg_gyro_z_s16 = BNO055_ZERO_U8X;
	double v_data_d = BNO055_ZERO_U8X;
	u8 v_gyro_unit_u8 = BNO055_ZERO_U8X;
	/* Read the current gyro unit and set the
	unit as dps if the unit is in rps */
	com_rslt = bno055_get_gyro_unit(&v_gyro_unit_u8);
	if (v_gyro_unit_u8 != GYRO_UNIT_DPS)
		com_rslt += bno055_set_gyro_unit(GYRO_UNIT_DPS);
		if (com_rslt == SUCCESS) {
			/* Read gyro raw z data */
			com_rslt += bno055_read_gyro_z(&v_reg_gyro_z_s16);
			if (com_rslt == SUCCESS) {
				/* Convert raw gyro z to dps */
				v_data_d =
				(double)(v_reg_gyro_z_s16/GYRO_DIV_DPS);
				*v_gyro_z_d = v_data_d;
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the gyro z raw data
 *	to rps output as double
 *
 *	@param v_gyro_z_d : The gyro z rps double data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_gyro_z_rps(
double *v_gyro_z_d)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	s16 v_reg_gyro_z_s16 = BNO055_ZERO_U8X;
	double v_data_d = BNO055_ZERO_U8X;
	u8 v_gyro_unit_u8 = BNO055_ZERO_U8X;
	/* Read the current gyro unit and set the
	unit as rps if the unit is in dps */
	com_rslt = bno055_get_gyro_unit(&v_gyro_unit_u8);
	if (v_gyro_unit_u8 != GYRO_UNIT_RPS)
		com_rslt += bno055_set_gyro_unit(GYRO_UNIT_RPS);
		if (com_rslt == SUCCESS) {
			/* Read gyro raw x data */
			com_rslt += bno055_read_gyro_z(&v_reg_gyro_z_s16);
			if (com_rslt == SUCCESS) {
				/* Convert raw gyro x to rps */
				v_data_d =
				(double)(v_reg_gyro_z_s16/GYRO_DIV_RPS);
				*v_gyro_z_d = v_data_d;
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the gyro xyz raw data
 *	to dps output as double
 *
 *	@param gyro_xyz : The dps data of gyro xyz
 *
 *	Parameter |    result
 *  --------- | -----------------
 *	 x        | dps data of gyro
 *	 y        | dps data of gyro
 *	 z        | dps data of gyro
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_gyro_xyz_dps(
struct bno055_gyro_double_t *gyro_xyz)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	struct bno055_gyro_t reg_gyro_xyz = {
	BNO055_ZERO_U8X, BNO055_ZERO_U8X, BNO055_ZERO_U8X};
	u8 v_gyro_unit_u8 = BNO055_ZERO_U8X;
	/* Read the current gyro unit and set the
	unit as dps if the unit is in rps */
	com_rslt = bno055_get_gyro_unit(&v_gyro_unit_u8);
	if (v_gyro_unit_u8 != GYRO_UNIT_DPS)
		com_rslt += bno055_set_gyro_unit(GYRO_UNIT_DPS);
		if (com_rslt == SUCCESS) {
			/* Read gyro raw xyz data */
			com_rslt += bno055_read_gyro_xyz(&reg_gyro_xyz);
			if (com_rslt == SUCCESS) {
				/* Convert gyro raw xyz to dps*/
				gyro_xyz->x =
				(double)(reg_gyro_xyz.x/GYRO_DIV_DPS);
				gyro_xyz->y =
				(double)(reg_gyro_xyz.y/GYRO_DIV_DPS);
				gyro_xyz->z =
				(double)(reg_gyro_xyz.z/GYRO_DIV_DPS);
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the gyro xyz raw data
 *	to rps output as double
 *
 *	@param gyro_xyz : The rps data of gyro xyz
 *
 *	Parameter |    result
 *  --------- | -----------------
 *	 x        | rps data of gyro
 *	 y        | rps data of gyro
 *	 z        | rps data of gyro
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_gyro_xyz_rps(
struct bno055_gyro_double_t *gyro_xyz)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	struct bno055_gyro_t reg_gyro_xyz = {
	BNO055_ZERO_U8X, BNO055_ZERO_U8X, BNO055_ZERO_U8X};
	u8 v_gyro_unit_u8 = BNO055_ZERO_U8X;
	/* Read the current gyro unit and set the
	unit as rps if the unit is in dps */
	com_rslt = bno055_get_gyro_unit(&v_gyro_unit_u8);
	if (v_gyro_unit_u8 != GYRO_UNIT_RPS)
		com_rslt += bno055_set_gyro_unit(GYRO_UNIT_RPS);
		if (com_rslt == SUCCESS) {
			/* Read gyro raw x data */
			com_rslt += bno055_read_gyro_xyz(&reg_gyro_xyz);
			if (com_rslt == SUCCESS) {
				/* Convert the raw gyro xyz to rps*/
				gyro_xyz->x =
				(double)(reg_gyro_xyz.x/GYRO_DIV_RPS);
				gyro_xyz->y =
				(double)(reg_gyro_xyz.y/GYRO_DIV_RPS);
				gyro_xyz->z =
				(double)(reg_gyro_xyz.z/GYRO_DIV_RPS);
			} else {
			com_rslt = ERROR;
			}
		} else {
			com_rslt = ERROR;
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the Euler h raw data
 *	to degree output as double
 *
 *	@param v_euler_h_d : The double value of Euler h degree
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_euler_h_deg(
double *v_euler_h_d)
{
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	s16 v_reg_euler_h_s16 = BNO055_ZERO_U8X;
	double v_data_d = BNO055_ZERO_U8X;
	u8 v_euler_unit_u8 = BNO055_ZERO_U8X;
	/* Read the current Euler unit and set the
	unit as degree if the unit is in radians */
	com_rslt = bno055_get_euler_unit(&v_euler_unit_u8);
	if (v_euler_unit_u8 != EULER_UNIT_DEG)
		com_rslt += bno055_set_euler_unit(EULER_UNIT_DEG);
		if (com_rslt ==  SUCCESS) {
			/* Read Euler raw h data*/
			com_rslt += bno055_read_euler_h(&v_reg_euler_h_s16);
			if (com_rslt == SUCCESS) {
				/* Convert raw Euler h to degree */
				v_data_d =
				(double)(v_reg_euler_h_s16/EULER_DIV_DEG);
				*v_euler_h_d = v_data_d;
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the Euler h raw data
 *	to radians output as double
 *
 *	@param v_euler_h_d : The double value of Euler h radians
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_euler_h_rad(
double *v_euler_h_d)
{
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	s16 v_reg_euler_h_s16 = BNO055_ZERO_U8X;
	double v_data_d = BNO055_ZERO_U8X;
	u8 v_euler_unit_u8 = BNO055_ZERO_U8X;
	/* Read the current Euler unit and set the
	unit as radians if the unit is in degree */
	com_rslt = bno055_get_euler_unit(&v_euler_unit_u8);
	if (v_euler_unit_u8 != EULER_UNIT_RAD)
		com_rslt += bno055_set_euler_unit(EULER_UNIT_RAD);
		if (com_rslt == SUCCESS) {
			/* Read Euler raw h data*/
			com_rslt += bno055_read_euler_h(&v_reg_euler_h_s16);
			if (com_rslt == SUCCESS) {
				/* Convert raw Euler h to radians */
				v_data_d =
				(double)(v_reg_euler_h_s16/EULER_DIV_RAD);
				*v_euler_h_d = v_data_d;
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the Euler r raw data
 *	to degree output as double
 *
 *	@param v_euler_r_d : The double value of Euler r degree
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_euler_r_deg(
double *v_euler_r_d)
{
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	s16 v_reg_euler_r_s16 = BNO055_ZERO_U8X;
	double v_data_d = BNO055_ZERO_U8X;
	u8 v_euler_unit_u8 = BNO055_ZERO_U8X;
	/* Read the current Euler unit and set the
	unit as degree if the unit is in radians */
	com_rslt = bno055_get_euler_unit(&v_euler_unit_u8);
	if (v_euler_unit_u8 != EULER_UNIT_DEG)
		com_rslt += bno055_set_euler_unit(EULER_UNIT_DEG);
		if (com_rslt ==  SUCCESS) {
			/* Read Euler raw r data*/
			com_rslt += bno055_read_euler_r(&v_reg_euler_r_s16);
			if (com_rslt == SUCCESS) {
				/* Convert raw Euler r to degree */
				v_data_d =
				(double)(v_reg_euler_r_s16/EULER_DIV_DEG);
				*v_euler_r_d = v_data_d;
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the Euler r raw data
 *	to radians output as double
 *
 *	@param v_euler_r_d : The double value of Euler r radians
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_euler_r_rad(
double *v_euler_r_d)
{
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	s16 v_reg_euler_r_s16 = BNO055_ZERO_U8X;
	double v_data_d = BNO055_ZERO_U8X;
	u8 v_euler_unit_u8 = BNO055_ZERO_U8X;
	/* Read the current Euler unit and set the
	unit as radians if the unit is in degree */
	com_rslt = bno055_get_euler_unit(&v_euler_unit_u8);
	if (v_euler_unit_u8 != EULER_UNIT_RAD)
		com_rslt += bno055_set_euler_unit(EULER_UNIT_RAD);
		if (com_rslt == SUCCESS) {
			/* Read Euler raw r data*/
			com_rslt += bno055_read_euler_r(&v_reg_euler_r_s16);
			if (com_rslt == SUCCESS) {
				/* Convert raw Euler r to radians */
				v_data_d =
				(double)(v_reg_euler_r_s16/EULER_DIV_RAD);
				*v_euler_r_d = v_data_d;
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the Euler p raw data
 *	to degree output as double
 *
 *	@param v_euler_p_d : The double value of Euler p degree
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_euler_p_deg(
double *v_euler_p_d)
{
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	s16 v_reg_euler_p_s16 = BNO055_ZERO_U8X;
	double v_data_d = BNO055_ZERO_U8X;
	u8 v_euler_unit_u8 = BNO055_ZERO_U8X;
	/* Read the current Euler unit and set the
	unit as degree if the unit is in radians */
	com_rslt = bno055_get_euler_unit(&v_euler_unit_u8);
	if (v_euler_unit_u8 != EULER_UNIT_DEG)
		com_rslt += bno055_set_euler_unit(EULER_UNIT_DEG);
		if (com_rslt ==  SUCCESS) {
			/* Read Euler raw p data*/
			com_rslt += bno055_read_euler_p(&v_reg_euler_p_s16);
			if (com_rslt == SUCCESS) {
				/* Convert raw Euler p to degree*/
				v_data_d =
				(double)(v_reg_euler_p_s16/EULER_DIV_DEG);
				*v_euler_p_d = v_data_d;
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the Euler p raw data
 *	to radians output as double
 *
 *	@param v_euler_p_d : The double value of Euler p radians
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */

BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_euler_p_rad(
double *v_euler_p_d)
{
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	s16 v_reg_euler_p_s16 = BNO055_ZERO_U8X;
	double v_data_d = BNO055_ZERO_U8X;
	u8 v_euler_unit_u8 = BNO055_ZERO_U8X;
	/* Read the current Euler unit and set the
	unit as radians if the unit is in degree */
	com_rslt = bno055_get_euler_unit(&v_euler_unit_u8);
	if (v_euler_unit_u8 != EULER_UNIT_RAD)
		com_rslt += bno055_set_euler_unit(EULER_UNIT_RAD);
		if (com_rslt == SUCCESS) {
			/* Read Euler raw p data*/
			com_rslt += bno055_read_euler_p(&v_reg_euler_p_s16);
			if (com_rslt == SUCCESS) {
				/* Convert raw p to radians*/
				v_data_d =
				(double)(v_reg_euler_p_s16/EULER_DIV_RAD);
				*v_euler_p_d = v_data_d;
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the Euler hpr raw data
 *	to degree output as double
 *
 *	@param euler_hpr : The degree data of Euler hpr
 *
 *	Parameter |    result
 *  --------- | -----------------
 *	 h        | degree data of Euler
 *	 r        | degree data of Euler
 *	 p        | degree data of Euler
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */

BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_euler_hpr_deg(
struct bno055_euler_double_t *euler_hpr)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	struct bno055_euler_t reg_euler = {BNO055_ZERO_U8X,
	BNO055_ZERO_U8X, BNO055_ZERO_U8X};
	u8 v_euler_unit_u8 = BNO055_ZERO_U8X;
	/* Read the current Euler unit and set the
	unit as degree if the unit is in radians */
	com_rslt = bno055_get_euler_unit(&v_euler_unit_u8);
	if (v_euler_unit_u8 != EULER_UNIT_DEG)
		com_rslt += bno055_set_euler_unit(EULER_UNIT_DEG);
	if (com_rslt == SUCCESS) {
		/* Read Euler raw h data*/
			com_rslt += bno055_read_euler_hrp(&reg_euler);
		if (com_rslt == SUCCESS) {
			/* Convert raw Euler hrp to degree*/
			euler_hpr->h =
			(double)(reg_euler.h/EULER_DIV_DEG);
			euler_hpr->p =
			(double)(reg_euler.p/EULER_DIV_DEG);
			euler_hpr->r =
			(double)(reg_euler.r/EULER_DIV_DEG);
		} else {
		com_rslt = ERROR;
		}
	} else {
	com_rslt = ERROR;
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the Euler hpr raw data
 *	to radians output as double
 *
 *	@param euler_hpr : The radians data of Euler hpr
 *
 *	Parameter |    result
 *  --------- | -----------------
 *	 h        | radians data of Euler
 *	 r        | radians data of Euler
 *	 p        | radians data of Euler
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */

BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_euler_hpr_rad(
struct bno055_euler_double_t *euler_hpr)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	struct bno055_euler_t reg_euler = {BNO055_ZERO_U8X,
	BNO055_ZERO_U8X, BNO055_ZERO_U8X};
	u8 v_euler_unit_u8 = BNO055_ZERO_U8X;
	/* Read the current Euler unit and set the
	unit as radians if the unit is in degree */
	com_rslt = bno055_get_euler_unit(&v_euler_unit_u8);
	if (v_euler_unit_u8 != EULER_UNIT_RAD)
		com_rslt = bno055_set_euler_unit(EULER_UNIT_RAD);
		if (com_rslt == SUCCESS) {
			/* Read the raw hrp */
			com_rslt = bno055_read_euler_hrp(&reg_euler);
			if (com_rslt == SUCCESS) {
				/* Convert raw Euler hrp to radians*/
				euler_hpr->h =
				(double)(reg_euler.h/EULER_DIV_RAD);
				euler_hpr->p =
				(double)(reg_euler.p/EULER_DIV_RAD);
				euler_hpr->r =
				(double)(reg_euler.r/EULER_DIV_RAD);
			} else {
			com_rslt = ERROR;
			}
		} else {
			com_rslt = ERROR;
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the linear
 *	accel x raw data to meterpersecseq output as double
 *
 *	@param v_linear_accel_x_d : The double value of
 *	linear accel x meterpersecseq
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */

BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_linear_accel_x_msq(
double *v_linear_accel_x_d)
{
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	s16 v_reg_linear_accel_x_s16 = BNO055_ZERO_U8X;
	double v_data_d = BNO055_ZERO_U8X;
	/* Read the raw x of linear accel */
	com_rslt = bno055_read_linear_accel_x(&v_reg_linear_accel_x_s16);
	if (com_rslt == SUCCESS) {
		/* Convert the raw x to m/s2 */
		v_data_d = (double)
		(v_reg_linear_accel_x_s16/LINEAR_ACCEL_DIV_MSQ);
		*v_linear_accel_x_d = v_data_d;
	} else {
		com_rslt = ERROR;
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the linear
 *	accel y raw data to meterpersecseq output as double
 *
 *	@param v_linear_accel_y_d : The double value of
 *	linear accel y meterpersecseq
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */

BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_linear_accel_y_msq(
double *v_linear_accel_y_d)
{
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	s16 v_reg_linear_accel_y_s16 = BNO055_ZERO_U8X;
	double v_data_d = BNO055_ZERO_U8X;
	/* Read the raw x of linear accel */
	com_rslt = bno055_read_linear_accel_y(&v_reg_linear_accel_y_s16);
	if (com_rslt == SUCCESS) {
		/* Convert the raw y to m/s2 */
		v_data_d = (double)
		(v_reg_linear_accel_y_s16/LINEAR_ACCEL_DIV_MSQ);
		*v_linear_accel_y_d = v_data_d;
	} else {
		com_rslt = ERROR;
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the linear
 *	accel z raw data to meterpersecseq output as double
 *
 *	@param v_linear_accel_z_d : The double value of
 *	linear accel z meterpersecseq
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */

BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_linear_accel_z_msq(
double *v_linear_accel_z_d)
{
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	s16 v_reg_linear_accel_z_s16 = BNO055_ZERO_U8X;
	double v_data_d = BNO055_ZERO_U8X;
	/* Read the raw x of linear accel */
	com_rslt = bno055_read_linear_accel_z(&v_reg_linear_accel_z_s16);
	if (com_rslt == SUCCESS) {
		/* Convert the raw z to m/s2 */
		v_data_d =
		(double)(v_reg_linear_accel_z_s16/LINEAR_ACCEL_DIV_MSQ);
		*v_linear_accel_z_d = v_data_d;
	} else {
		com_rslt = ERROR;
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the linear accel xyz raw data
 *	to meterpersecseq output as double
 *
 *	@param linear_accel_xyz : The meterpersecseq data of linear accel xyz
 *
 *	Parameter |    result
 *  --------- | -----------------
 *	 x        | meterpersecseq data of linear accel
 *	 y        | meterpersecseq data of linear accel
 *	 z        | meterpersecseq data of linear accel
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */

BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_linear_accel_xyz_msq(
struct bno055_linear_accel_double_t *linear_accel_xyz)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	struct bno055_linear_accel_t reg_linear_accel_xyz = {
	BNO055_ZERO_U8X, BNO055_ZERO_U8X, BNO055_ZERO_U8X};
	/* Read the raw xyz of linear accel */
	com_rslt = bno055_read_linear_accel_xyz(&reg_linear_accel_xyz);
	if (com_rslt == SUCCESS) {
		/* Convert the raw xyz of linear accel to m/s2 */
		linear_accel_xyz->x =
		(double)(reg_linear_accel_xyz.x/LINEAR_ACCEL_DIV_MSQ);
		linear_accel_xyz->y =
		(double)(reg_linear_accel_xyz.y/LINEAR_ACCEL_DIV_MSQ);
		linear_accel_xyz->z =
		(double)(reg_linear_accel_xyz.z/LINEAR_ACCEL_DIV_MSQ);
	} else {
		com_rslt = ERROR;
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the gravity
 *	x raw data to meterpersecseq output as double
 *
 *	@param v_gravity_x_d : The double value of gravity x meterpersecseq
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_gravity_double_x_msq(
double *v_gravity_x_d)
{
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	s16 v_reg_gravity_x_s16 = BNO055_ZERO_U8X;
	double v_data_d = BNO055_ZERO_U8X;
	/* Read raw gravity of x*/
	com_rslt = bno055_read_gravity_x(&v_reg_gravity_x_s16);
	if (com_rslt == SUCCESS) {
		/* Convert raw gravity of x to m/s2 */
		v_data_d =
		(double)(v_reg_gravity_x_s16/GRAVITY_DIV_MSQ);
		*v_gravity_x_d = v_data_d;
	} else {
		com_rslt = ERROR;
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the gravity
 *	y raw data to meterpersecseq output as double
 *
 *	@param v_gravity_y_d : The double value of gravity y meterpersecseq
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_gravity_double_y_msq(
double *v_gravity_y_d)
{
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	s16 v_reg_gravity_y_s16 = BNO055_ZERO_U8X;
	double v_data_d = BNO055_ZERO_U8X;
	/* Read raw gravity of y */
	com_rslt = bno055_read_gravity_y(&v_reg_gravity_y_s16);
	if (com_rslt == SUCCESS) {
		/* convert raw gravity of y to m/s2 */
		v_data_d =
		(double)(v_reg_gravity_y_s16/GRAVITY_DIV_MSQ);
		*v_gravity_y_d = v_data_d;
	} else {
		com_rslt += ERROR;
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the gravity
 *	z raw data to meterpersecseq output as double
 *
 *	@param v_gravity_z_d : The double value of gravity z meterpersecseq
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_gravity_double_z_msq(
double *v_gravity_z_d)
{
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	s16 v_reg_gravity_z_s16 = BNO055_ZERO_U8X;
	double v_data_d = BNO055_ZERO_U8X;
	/* Read raw gravity of z */
	com_rslt = bno055_read_gravity_z(&v_reg_gravity_z_s16);
	if (com_rslt == SUCCESS) {
		/* Convert raw gravity of z to m/s2 */
		v_data_d =
		(double)(v_reg_gravity_z_s16/GRAVITY_DIV_MSQ);
		*v_gravity_z_d = v_data_d;
	} else {
		com_rslt += ERROR;
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the gravity xyz raw data
 *	to meterpersecseq output as double
 *
 *	@param gravity_xyz : The meterpersecseq data of gravity xyz
 *
 *	Parameter |    result
 *  --------- | -----------------
 *	 x        | meterpersecseq data of gravity
 *	 y        | meterpersecseq data of gravity
 *	 z        | meterpersecseq data of gravity
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_gravity_xyz_msq(
struct bno055_gravity_double_t *gravity_xyz)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	struct bno055_gravity_t reg_gravity_xyz = {
	BNO055_ZERO_U8X, BNO055_ZERO_U8X, BNO055_ZERO_U8X};
	/* Read raw gravity of xyz */
	com_rslt = bno055_read_gravity_xyz(&reg_gravity_xyz);
	if (com_rslt == SUCCESS) {
		/* Convert raw gravity of xyz to m/s2 */
		gravity_xyz->x =
		(double)(reg_gravity_xyz.x/GRAVITY_DIV_MSQ);
		gravity_xyz->y =
		(double)(reg_gravity_xyz.y/GRAVITY_DIV_MSQ);
		gravity_xyz->z =
		(double)(reg_gravity_xyz.z/GRAVITY_DIV_MSQ);
	} else {
		com_rslt += ERROR;
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the temperature
 *	data to Fahrenheit output as double
 *
 *	@param v_temp_d : The double value of temperature Fahrenheit
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_temp_fahrenheit(
double *v_temp_d)
{
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	s8 v_reg_temp_s8 = BNO055_ZERO_U8X;
	double v_data_d = BNO055_ZERO_U8X;
	u8 v_temp_unit_u8 = BNO055_ZERO_U8X;
	/* Read the current temperature unit and set the
	unit as Fahrenheit if the unit is in Celsius */
	com_rslt = bno055_get_temp_unit(&v_temp_unit_u8);
	if (v_temp_unit_u8 != TEMP_UNIT_FAHRENHEIT)
		com_rslt += bno055_set_temp_unit(TEMP_UNIT_FAHRENHEIT);
		if (com_rslt == SUCCESS) {
			/* Read the raw temperature data */
			com_rslt += bno055_read_temp_data(&v_reg_temp_s8);
			if (com_rslt == SUCCESS) {
				/* Convert raw temperature data to Fahrenheit*/
				v_data_d =
				(double)(v_reg_temp_s8/TEMP_DIV_FAHRENHEIT);
				*v_temp_d = v_data_d;
			} else {
			com_rslt += ERROR;
			}
		} else {
		com_rslt += ERROR;
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to convert the temperature
 *	data to Celsius output as double
 *
 *	@param v_temp_d : The double value of temperature Celsius
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_temp_celsius(
double *v_temp_d)
{
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	s8 v_reg_temp_s8 = BNO055_ZERO_U8X;
	double v_data_d = BNO055_ZERO_U8X;
	u8 v_temp_unit_u8 = BNO055_ZERO_U8X;
	/* Read the current temperature unit and set the
	unit as Fahrenheit if the unit is in Celsius */
	com_rslt = bno055_get_temp_unit(&v_temp_unit_u8);
	if (v_temp_unit_u8 != TEMP_UNIT_CELSIUS)
		com_rslt += bno055_set_temp_unit(TEMP_UNIT_CELSIUS);
		if (com_rslt == SUCCESS) {
			/* Read the raw temperature data */
			com_rslt += bno055_read_temp_data(&v_reg_temp_s8);
			if (com_rslt == SUCCESS) {
				/* Convert raw temperature data to Fahrenheit*/
				v_data_d =
				(double)(v_reg_temp_s8/TEMP_DIV_CELSIUS);
				*v_temp_d = v_data_d;
			} else {
			com_rslt += ERROR;
			}
		} else {
		com_rslt += ERROR;
		}
	return com_rslt;
}
#endif
/*!
 *	@brief This API used to read
 *	mag calibration status from register from 0x35 bit 0 and 1
 *
 *	@param v_mag_calib_u8 : The value of mag calib status
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_mag_calib_stat(
u8 *v_mag_calib_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, mag calib
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read the mag calib v_stat_s8 */
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_MAG_CALIB_STAT__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_mag_calib_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_MAG_CALIB_STAT);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to read
 *	accel calibration status from register from 0x35 bit 2 and 3
 *
 *	@param v_accel_calib_u8 : The value of accel calib status
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_calib_stat(
u8 *v_accel_calib_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty*/
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel calib
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read the accel calib v_stat_s8 */
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_CALIB_STAT__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_accel_calib_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_ACCEL_CALIB_STAT);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to read
 *	gyro calibration status from register from 0x35 bit 4 and 5
 *
 *	@param v_gyro_calib_u8 : The value of gyro calib status
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_calib_stat(
u8 *v_gyro_calib_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, gyro calib
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read the gyro calib status */
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_CALIB_STAT__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_gyro_calib_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_GYRO_CALIB_STAT);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to read
 *	system calibration status from register from 0x35 bit 6 and 7
 *
 *	@param v_sys_calib_u8 : The value of system calib status
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_sys_calib_stat(
u8 *v_sys_calib_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty*/
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page,system calib
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read the system calib */
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_SYS_CALIB_STAT__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_sys_calib_u8 =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_SYS_CALIB_STAT);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to read
 *	self test of accel from register from 0x36 bit 0
 *
 *	@param v_selftest_accel_u8 : The value of self test of accel
 *
 *    v_selftest_accel_u8 |  result
 *   -------------------- | ---------------------
 *     0x00               | indicates test failed
 *     0x01               | indicated test passed
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_selftest_accel(
u8 *v_selftest_accel_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel self test is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read the accel self test */
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_SELFTEST_ACCEL__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_selftest_accel_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_SELFTEST_ACCEL);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to read
 *	self test of mag from register from 0x36 bit 1
 *
 *	@param v_selftest_mag_u8 : The value of self test of mag
 *
 *     v_selftest_mag_u8  |  result
 *   -------------------- | ---------------------
 *     0x00               | indicates test failed
 *     0x01               | indicated test passed
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_selftest_mag(
u8 *v_selftest_mag_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, self test of mag is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read the mag self test */
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_SELFTEST_MAG__REG, &v_data_u8r, BNO055_ONE_U8X);
			*v_selftest_mag_u8 =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_SELFTEST_MAG);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to read
 *	self test of gyro from register from 0x36 bit 2
 *
 *	@param v_selftest_gyro_u8 : The value of self test of gyro
 *
 *     v_selftest_gyro_u8 |  result
 *   -------------------- | ---------------------
 *     0x00               | indicates test failed
 *     0x01               | indicated test passed
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_selftest_gyro(
u8 *v_selftest_gyro_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page self test of gyro is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read the gyro self test */
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_SELFTEST_GYRO__REG, &v_data_u8r, BNO055_ONE_U8X);
			*v_selftest_gyro_u8 =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_SELFTEST_GYRO);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to read
 *	self test of micro controller from register from 0x36 bit 3
 *
 *	@param v_selftest_mcu_u8 : The value of self test of micro controller
 *
 *     v_selftest_mcu_u8  |  result
 *   -------------------- | ---------------------
 *     0x00               | indicates test failed
 *     0x01               | indicated test passed
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_selftest_mcu(
u8 *v_selftest_mcu_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page self test of micro controller
		is available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read the self test of micro controller*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_SELFTEST_MCU__REG, &v_data_u8r, BNO055_ONE_U8X);
			*v_selftest_mcu_u8 =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_SELFTEST_MCU);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to read the v_stat_s8 of
 *	gyro anymotion interrupt from register from 0x37 bit 2
 *
 *	@param v_gyro_any_motion_u8 : The value of gyro anymotion interrupt
 *
 *     v_gyro_any_motion_u8  |  result
 *    --------------------   | ---------------------
 *     0x00                  | indicates no interrupt triggered
 *     0x01                  | indicates interrupt triggered
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note Gyro anymotion interrupt can be enabled
 *	by the following APIs
 *
 *	bno055_set_intr_mask_gyro_any_motion()
 *
 *	bno055_set_intr_gyro_any_motion()
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_intr_stat_gyro_any_motion(
u8 *v_gyro_any_motion_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, gyro anymotion interrupt
		status is available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read the gyro anymotion interrupt v_stat_s8*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_INTR_STAT_GYRO_ANY_MOTION__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_gyro_any_motion_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_INTR_STAT_GYRO_ANY_MOTION);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to read the v_stat_s8 of
 *	gyro highrate interrupt from register from 0x37 bit 3
 *
 *	@param v_gyro_highrate_u8 : The value of gyro highrate interrupt
 *
 *     v_gyro_highrate_u8   |  result
 *    -------------------   | ---------------------
 *     0x00                 | indicates no interrupt triggered
 *     0x01                 | indicates interrupt triggered
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note Gyro highrate interrupt can be configured
 *			by the following APIs
 *
 *	bno055_set_intr_mask_gyro_highrate()
 *
 *	bno055_set_intr_gyro_highrate()
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_intr_stat_gyro_highrate(
u8 *v_gyro_highrate_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, gyro highrate is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read the gyro highrate interrupt v_stat_s8*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_INTR_STAT_GYRO_HIGHRATE__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_gyro_highrate_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_INTR_STAT_GYRO_HIGHRATE);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to read the v_stat_s8 of
 *	accel highg interrupt from register from 0x37 bit 5
 *
 *	@param v_accel_high_g_u8 : The value of accel highg interrupt
 *
 *     v_accel_high_g_u8    |  result
 *    -------------------   | ---------------------
 *     0x00                 | indicates no interrupt triggered
 *     0x01                 | indicates interrupt triggered
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note Accel highg interrupt can be configured
 *			by the following APIs
 *
 *	bno055_set_intr_mask_accel_high_g()
 *
 *	bno055_set_intr_accel_high_g()
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_intr_stat_accel_high_g(
u8 *v_accel_high_g_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel highg is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read the accel highg interrupt v_stat_s8 */
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_INTR_STAT_ACCEL_HIGH_G__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_accel_high_g_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_INTR_STAT_ACCEL_HIGH_G);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to read the v_stat_s8 of
 *	accel anymotion interrupt from register from 0x37 bit 6
 *
 *	@param v_accel_any_motion_u8 : The value of accel anymotion interrupt
 *
 *     v_accel_any_motion_u8 |  result
 *    -------------------   | ---------------------
 *     0x00                 | indicates no interrupt triggered
 *     0x01                 | indicates interrupt triggered
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note Accel anymotion interrupt can be configured
 *			by the following APIs
 *
 *	bno055_set_intr_mask_accel_any_motion()
 *
 *	bno055_set_intr_accel_any_motion()
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_intr_stat_accel_any_motion(
u8 *v_accel_any_motion_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel anymotion is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read the accel anymotion interrupt v_stat_s8 */
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_INTR_STAT_ACCEL_ANY_MOTION__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_accel_any_motion_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_INTR_STAT_ACCEL_ANY_MOTION);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to read the v_stat_s8 of
 *	accel nomotion/slowmotion interrupt from register from 0x37 bit 6
 *
 *	@param v_accel_no_motion_u8 : The value of accel
 *	nomotion/slowmotion interrupt
 *
 *     v_accel_no_motion_u8 |  result
 *    -------------------   | ---------------------
 *     0x00                 | indicates no interrupt triggered
 *     0x01                 | indicates interrupt triggered
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note Accel nomotion/slowmotion interrupt can be configured
 *			by the following APIs
 *
 *	bno055_set_intr_mask_accel_nomotion()
 *
 *	bno055_set_intr_accel_nomotion()
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_intr_stat_accel_no_motion(
u8 *v_accel_no_motion_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel
		nomotion/slowmotion interrupt
		is available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read the v_stat_s8 of accel
			nomotion/slowmotion interrupt*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_INTR_STAT_ACCEL_NO_MOTION__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_accel_no_motion_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_INTR_STAT_ACCEL_NO_MOTION);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to read status of main clock
 *	from the register 0x38 bit 0
 *
 *	@param v_stat_main_clk_u8 : the status of main clock
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_stat_main_clk(
u8 *v_stat_main_clk_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, status of main clk is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read the status of main clk */
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_SYS_MAIN_CLK__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_stat_main_clk_u8 =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_SYS_MAIN_CLK);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to read system status
 *	code from the register 0x39 it is a byte of data
 *
 *	@param v_sys_stat_u8 : the status of system
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_sys_stat_code(
u8 *v_sys_stat_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, the status of system is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read the the status of system*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_SYS_STAT_CODE__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_sys_stat_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_SYS_STAT_CODE);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to read system error
 *	code from the register 0x3A it is a byte of data
 *
 *	@param v_sys_error_u8 : The value of system error code
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_sys_error_code(
u8 *v_sys_error_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, system error code is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read the system error code*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_SYS_ERROR_CODE__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_sys_error_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_SYS_ERROR_CODE);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to read the accel unit
 *	from register from 0x3B bit 0
 *
 *	@param v_accel_unit_u8 : The value of accel unit
 *
 *    v_accel_unit_u8 |   result
 *   -------------    | ---------------
 *        0x00        | ACCEL_UNIT_MSQ
 *        0x01        | ACCEL_UNIT_MG
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_unit(
u8 *v_accel_unit_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel unit is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read the accel unit */
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_UNIT__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_accel_unit_u8 =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_ACCEL_UNIT);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to write the accel unit
 *	from register from 0x3B bit 0
 *
 *	@param v_accel_unit_u8 : The value of accel unit
 *
 *    v_accel_unit_u8 |   result
 *   -------------    | ---------------
 *        0x00        | ACCEL_UNIT_MSQ
 *        0x01        | ACCEL_UNIT_MG
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_unit(
u8 v_accel_unit_u8)
{
BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 v_data_u8r = BNO055_ZERO_U8X;
u8 v_prev_opmode_u8 = OPERATION_MODE_CONFIG;
s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
	} else {
	/* The write operation effective only if the operation
	mode is in config mode, this part of code is checking the
	current operation mode and set the config mode */
	v_stat_s8 = bno055_get_operation_mode(&v_prev_opmode_u8);
	if (v_stat_s8 == SUCCESS) {
		if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
			v_stat_s8 += bno055_set_operation_mode
			(OPERATION_MODE_CONFIG);
			if (v_stat_s8 == SUCCESS) {
				/* Write the accel unit */
				com_rslt = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_ACCEL_UNIT__REG,
				&v_data_u8r, BNO055_ONE_U8X);
				if (com_rslt == SUCCESS) {
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_ACCEL_UNIT, v_accel_unit_u8);
					com_rslt +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_ACCEL_UNIT__REG,
					&v_data_u8r, BNO055_ONE_U8X);
				}
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	}
	if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
		/* set the operation mode
		of previous operation mode*/
		com_rslt += bno055_set_operation_mode
		(v_prev_opmode_u8);
	return com_rslt;
}
/*!
 *	@brief This API used to read the gyro unit
 *	from register from 0x3B bit 1
 *
 *	@param v_gyro_unit_u8 : The value of accel unit
 *
 *	v_gyro_unit_u8  |  result
 *	-------------   | -----------
 *    0x00          | GYRO_UNIT_DPS
 *    0x01          | GYRO_UNIT_RPS
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_unit(
u8 *v_gyro_unit_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, gyro unit is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read the gyro unit */
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_UNIT__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_gyro_unit_u8 =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_GYRO_UNIT);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to write the gyro unit
 *	from register from 0x3B bit 1
 *
 *	@param v_gyro_unit_u8 : The value of accel unit
 *
 *	v_gyro_unit_u8  |  result
 *	-------------   | -----------
 *    0x00          | GYRO_UNIT_DPS
 *    0x01          | GYRO_UNIT_RPS
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_unit(u8 v_gyro_unit_u8)
{
BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 v_data_u8r = BNO055_ZERO_U8X;
u8 v_prev_opmode_u8 = OPERATION_MODE_CONFIG;
s8 v_stat_s8 = ERROR;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
	} else {
	/* The write operation effective only if the operation
	mode is in config mode, this part of code is checking the
	current operation mode and set the config mode */
	v_stat_s8 = bno055_get_operation_mode(&v_prev_opmode_u8);
	if (v_stat_s8 == SUCCESS) {
		if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
			v_stat_s8 += bno055_set_operation_mode
			(OPERATION_MODE_CONFIG);
			if (v_stat_s8 == SUCCESS) {
				/* Write the gyro unit */
				com_rslt = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_GYRO_UNIT__REG,
				&v_data_u8r, BNO055_ONE_U8X);
				if (com_rslt == SUCCESS) {
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_GYRO_UNIT, v_gyro_unit_u8);
					com_rslt +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_GYRO_UNIT__REG,
					&v_data_u8r, BNO055_ONE_U8X);
				}
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	}
	if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
		/* set the operation mode
		of previous operation mode*/
		com_rslt += bno055_set_operation_mode
		(v_prev_opmode_u8);
	return com_rslt;
}
/*!
 *	@brief This API used to read the Euler unit
 *	from register from 0x3B bit 2
 *
 *	@param v_euler_unit_u8 : The value of accel unit
 *
 *    v_euler_unit_u8 | result
 *   --------------   | -----------
 *      0x00          | EULER_UNIT_DEG
 *      0x01          | EULER_UNIT_RAD
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_euler_unit(
u8 *v_euler_unit_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, Euler unit is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read the Euler unit */
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_EULER_UNIT__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_euler_unit_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_EULER_UNIT);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to write the Euler unit
 *	from register from 0x3B bit 2
 *
 *	@param v_euler_unit_u8 : The value of Euler unit
 *
 *    v_euler_unit_u8 | result
 *   --------------   | -----------
 *      0x00          | EULER_UNIT_DEG
 *      0x01          | EULER_UNIT_RAD
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_euler_unit(u8 v_euler_unit_u8)
{
BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 v_data_u8r = BNO055_ZERO_U8X;
u8 v_prev_opmode_u8 = OPERATION_MODE_CONFIG;
s8 v_stat_s8 = ERROR;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
	} else {
		/* The write operation effective only if the operation
		mode is in config mode, this part of code is checking the
		current operation mode and set the config mode */
		v_stat_s8 = bno055_get_operation_mode(&v_prev_opmode_u8);
		if (v_stat_s8 == SUCCESS) {
			if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
				v_stat_s8 += bno055_set_operation_mode
				(OPERATION_MODE_CONFIG);
			if (v_stat_s8 == SUCCESS) {
				/* Write the Euler unit*/
				com_rslt = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_EULER_UNIT__REG,
				&v_data_u8r, BNO055_ONE_U8X);
				if (com_rslt == SUCCESS) {
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_EULER_UNIT, v_euler_unit_u8);
					com_rslt +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_EULER_UNIT__REG,
					&v_data_u8r, BNO055_ONE_U8X);
				}
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	}
	if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
		/* set the operation mode
		of previous operation mode*/
		com_rslt += bno055_set_operation_mode
		(v_prev_opmode_u8);
	return com_rslt;
}
/*!
 *	@brief This API used to write the tilt unit
 *	from register from 0x3B bit 3
 *
 *	@param v_tilt_unit_u8 : The value of tilt unit
 *
 *    v_tilt_unit_u8  | result
 *   ---------------  | ---------
 *     0x00           | degrees
 *     0x01           | radians
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_tilt_unit(
u8 *v_tilt_unit_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_TILT_UNIT__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_tilt_unit_u8 =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_TILT_UNIT);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to write the tilt unit
 *	from register from 0x3B bit 3
 *
 *	@param v_tilt_unit_u8 : The value of tilt unit
 *
 *    v_tilt_unit_u8  | result
 *   ---------------  | ---------
 *     0x00           | degrees
 *     0x01           | radians
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 *
 *  \return Communication results
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_tilt_unit(u8 v_tilt_unit_u8)
{
BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 v_data_u8r = BNO055_ZERO_U8X;
u8 v_prev_opmode_u8 = OPERATION_MODE_CONFIG;
s8 v_stat_s8 = ERROR;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
	} else {
		/* The write operation effective only if the operation
		mode is in config mode, this part of code is checking the
		current operation mode and set the config mode */
		v_stat_s8 = bno055_get_operation_mode(&v_prev_opmode_u8);
		if (v_stat_s8 == SUCCESS) {
			if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
				v_stat_s8 += bno055_set_operation_mode
				(OPERATION_MODE_CONFIG);
			if (v_stat_s8 == SUCCESS) {
				com_rslt = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_TILT_UNIT__REG,
				&v_data_u8r, BNO055_ONE_U8X);
				if (com_rslt == SUCCESS) {
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_TILT_UNIT, v_tilt_unit_u8);
					com_rslt +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_TILT_UNIT__REG,
					&v_data_u8r, BNO055_ONE_U8X);
				}
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	}
	if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
		/* set the operation mode
		of previous operation mode*/
		com_rslt += bno055_set_operation_mode
		(v_prev_opmode_u8);
	return com_rslt;
}
/*!
 *	@brief This API used to read the temperature unit
 *	from register from 0x3B bit 4
 *
 *	@param v_temp_unit_u8 : The value of temperature unit
 *
 *    v_temp_unit_u8  |  result
 *   -----------      | --------------
 *      0x00          | TEMP_UNIT_CELCIUS
 *      0x01          | TEMP_UNIT_FAHRENHEIT
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_temp_unit(
u8 *v_temp_unit_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, temperature unit is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read the temperature unit */
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_TEMP_UNIT__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_temp_unit_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_TEMP_UNIT);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to write the temperature unit
 *	from register from 0x3B bit 4
 *
 *	@param v_temp_unit_u8 : The value of temperature unit
 *
 *    v_temp_unit_u8  |  result
 *   -----------      | --------------
 *      0x00          | TEMP_UNIT_CELCIUS
 *      0x01          | TEMP_UNIT_FAHRENHEIT
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_temp_unit(
u8 v_temp_unit_u8)
{
BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 v_data_u8r = BNO055_ZERO_U8X;
u8 v_prev_opmode_u8 = OPERATION_MODE_CONFIG;
s8 v_stat_s8 = ERROR;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
	} else {
		/* The write operation effective only if the operation
		mode is in config mode, this part of code is checking the
		current operation mode and set the config mode */
		v_stat_s8 = bno055_get_operation_mode(&v_prev_opmode_u8);
		if (v_stat_s8 == SUCCESS) {
			if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
				v_stat_s8 += bno055_set_operation_mode
				(OPERATION_MODE_CONFIG);
			if (v_stat_s8 == SUCCESS) {
				/* Write the temperature unit */
				com_rslt = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_TEMP_UNIT__REG,
				&v_data_u8r, BNO055_ONE_U8X);
				if (com_rslt == SUCCESS) {
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_TEMP_UNIT,
					v_temp_unit_u8);
					com_rslt +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_TEMP_UNIT__REG,
					&v_data_u8r, BNO055_ONE_U8X);
				}
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	}
	if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
		/* set the operation mode
		of previous operation mode*/
		com_rslt += bno055_set_operation_mode
		(v_prev_opmode_u8);
	return com_rslt;
}
/*!
 *	@brief This API used to read the current selected orientation mode
 *	from register from 0x3B bit 7
 *
 *	@param v_data_output_format_u8 : The value of data output format
 *
 *	  v_data_output_format_u8  | result
 *   --------------------      | --------
 *    0x00                     | Windows
 *    0x01                     | Android
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_data_output_format(
u8 *v_data_output_format_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, data output format is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read the data output format */
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_DATA_OUTPUT_FORMAT__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_data_output_format_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_DATA_OUTPUT_FORMAT);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to write the current selected orientation mode
 *	from register from 0x3B bit 7
 *
 *	@param v_data_output_format_u8 : The value of data output format
 *
 *	  v_data_output_format_u8  | result
 *   --------------------      | --------
 *    0x00                     | Windows
 *    0x01                     | Android
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_data_output_format(
u8 v_data_output_format_u8)
{
BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 v_data_u8r = BNO055_ZERO_U8X;
u8 v_prev_opmode_u8 = OPERATION_MODE_CONFIG;
s8 v_stat_s8 = ERROR;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
	} else {
		/* The write operation effective only if the operation
		mode is in config mode, this part of code is checking the
		current operation mode and set the config mode */
		v_stat_s8 = bno055_get_operation_mode(&v_prev_opmode_u8);
		if (v_stat_s8 == SUCCESS) {
			if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
				v_stat_s8 += bno055_set_operation_mode
				(OPERATION_MODE_CONFIG);
			if (v_stat_s8 == SUCCESS) {
				/* Write the data output format */
				com_rslt = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_DATA_OUTPUT_FORMAT__REG,
				&v_data_u8r, BNO055_ONE_U8X);
				if (com_rslt == SUCCESS) {
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_DATA_OUTPUT_FORMAT,
					v_data_output_format_u8);
					com_rslt +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_DATA_OUTPUT_FORMAT__REG,
					&v_data_u8r, BNO055_ONE_U8X);
				}
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	}
	if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
		/* set the operation mode
		of previous operation mode*/
		com_rslt += bno055_set_operation_mode
		(v_prev_opmode_u8);
	return com_rslt;
}
/*!	@brief This API used to read the operation mode
 *	from register from 0x3D bit 0 to 3
 *
 *	@param v_operation_mode_u8 : The value of operation mode
 *
 * v_operation_mode_u8 |      result      | comments
 * ----------|----------------------------|----------------------------
 *  0x00     | OPERATION_MODE_CONFIG      | Configuration mode
 *  0x01     | OPERATION_MODE_ACCONLY     | Reads accel data alone
 *  0x02     | OPERATION_MODE_MAGONLY     | Reads mag data alone
 *  0x03     | OPERATION_MODE_GYRONLY     | Reads gyro data alone
 *  0x04     | OPERATION_MODE_ACCMAG      | Reads accel and mag data
 *  0x05     | OPERATION_MODE_ACCGYRO     | Reads accel and gyro data
 *  0x06     | OPERATION_MODE_MAGGYRO     | Reads accel and mag data
 *  0x07     | OPERATION_MODE_ANY_MOTION  | Reads accel mag and gyro data
 *  0x08     | OPERATION_MODE_IMUPLUS     | Inertial measurement unit
 *   -       |       -                    | Reads accel,gyro and fusion data
 *  0x09     | OPERATION_MODE_COMPASS     | Reads accel, mag data
 *   -       |       -                    | and fusion data
 *  0x0A     | OPERATION_MODE_M4G         | Reads accel, mag data
 *    -      |       -                    | and fusion data
 *  0x0B     | OPERATION_MODE_NDOF_FMC_OFF| Nine degrees of freedom with
 *   -       |       -                    | fast magnetic calibration
 *   -       |       -                    | Reads accel,mag, gyro
 *   -       |       -                    | and fusion data
 *  0x0C     | OPERATION_MODE_NDOF        | Nine degrees of freedom
 *   -       |       -                    | Reads accel,mag, gyro
 *   -       |       -                    | and fusion data
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note In the config mode, all sensor and fusion data
 *	becomes zero and it is mainly derived
 *	to configure the various settings of the BNO
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_operation_mode(
u8 *v_operation_mode_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, operation mode is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read the value of operation mode*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_OPERATION_MODE__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_operation_mode_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_OPERATION_MODE);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!	@brief This API used to write the operation mode
 *	from register from 0x3D bit 0 to 3
 *
 *	@param v_operation_mode_u8 : The value of operation mode
 *
 *  v_operation_mode_u8  |      result    | comments
 * ----------|----------------------------|----------------------------
 *  0x00     | OPERATION_MODE_CONFIG      | Configuration mode
 *  0x01     | OPERATION_MODE_ACCONLY     | Reads accel data alone
 *  0x02     | OPERATION_MODE_MAGONLY     | Reads mag data alone
 *  0x03     | OPERATION_MODE_GYRONLY     | Reads gyro data alone
 *  0x04     | OPERATION_MODE_ACCMAG      | Reads accel and mag data
 *  0x05     | OPERATION_MODE_ACCGYRO     | Reads accel and gyro data
 *  0x06     | OPERATION_MODE_MAGGYRO     | Reads accel and mag data
 *  0x07     | OPERATION_MODE_ANY_MOTION  | Reads accel mag and gyro data
 *  0x08     | OPERATION_MODE_IMUPLUS     | Inertial measurement unit
 *   -       |       -                    | Reads accel,gyro and fusion data
 *  0x09     | OPERATION_MODE_COMPASS     | Reads accel, mag data
 *   -       |       -                    | and fusion data
 *  0x0A     | OPERATION_MODE_M4G         | Reads accel, mag data
 *    -      |       -                    | and fusion data
 *  0x0B     | OPERATION_MODE_NDOF_FMC_OFF| Nine degrees of freedom with
 *   -       |       -                    | fast magnetic calibration
 *   -       |       -                    | Reads accel,mag, gyro
 *   -       |       -                    | and fusion data
 *  0x0C     | OPERATION_MODE_NDOF        | Nine degrees of freedom
 *   -       |       -                    | Reads accel,mag, gyro
 *   -       |       -                    | and fusion data
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note In the config mode, all sensor and fusion data
 *	becomes zero and it is mainly derived
 *	to configure the various settings of the BNO
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_operation_mode(u8 v_operation_mode_u8)
{
BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 v_data_u8r = BNO055_ZERO_U8X;
u8 v_prev_opmode_u8 = OPERATION_MODE_CONFIG;
s8 v_stat_s8 = ERROR;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
	} else {
		/* The write operation effective only if the operation
		mode is in config mode, this part of code is checking the
		current operation mode and set the config mode */
		v_stat_s8 = bno055_get_operation_mode(&v_prev_opmode_u8);
		if (v_stat_s8 == SUCCESS) {
			/* If the previous operation mode is config it is
				directly write the operation mode */
			if (v_prev_opmode_u8 == OPERATION_MODE_CONFIG) {
				com_rslt = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_OPERATION_MODE__REG,
				&v_data_u8r, BNO055_ONE_U8X);
				if (com_rslt == SUCCESS) {
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_OPERATION_MODE,
					v_operation_mode_u8);
					com_rslt +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_OPERATION_MODE__REG,
					&v_data_u8r, BNO055_ONE_U8X);
					/* Config mode to other
					operation mode switching
					required delay of 600ms*/
					p_bno055->delay_msec(
						BNO055_SIX_HUNDRES_U8X);
				}
			} else {
				/* If the previous operation
				mode is not config it is
				 write the config mode */
				com_rslt = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_OPERATION_MODE__REG,
				&v_data_u8r, BNO055_ONE_U8X);
				if (com_rslt == SUCCESS) {
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_OPERATION_MODE,
					OPERATION_MODE_CONFIG);
					com_rslt += bno055_write_register(
					BNO055_OPERATION_MODE__REG,
					&v_data_u8r, BNO055_ONE_U8X);
					/* other mode to config mode switching
					required delay of 20ms*/
					p_bno055->delay_msec(BNO055_TWENTY_U8X);
				}
				/* Write the operation mode */
				if (v_operation_mode_u8 !=
				OPERATION_MODE_CONFIG) {
					com_rslt =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_OPERATION_MODE__REG,
					&v_data_u8r, BNO055_ONE_U8X);
					if (com_rslt == SUCCESS) {
						v_data_u8r = BNO055_SET_BITSLICE
						(v_data_u8r,
						BNO055_OPERATION_MODE,
						v_operation_mode_u8);
						com_rslt +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_OPERATION_MODE__REG,
						&v_data_u8r, BNO055_ONE_U8X);
						/* Config mode to other
						operation mode switching
						required delay of 600ms*/
						p_bno055->delay_msec(
						BNO055_SIX_HUNDRES_U8X);
					}
				}
			}
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!	@brief This API used to read the power mode
 *	from register from 0x3E bit 0 to 1
 *
 *	@param v_power_mode_u8 : The value of power mode
 *
 * v_power_mode_u8|      result    | comments
 * ----------|---------------------|-------------------------------------
 *  0x00     | POWER_MODE_NORMAL   | In the NORMAL mode the register
 *    -      |       -             | map and the internal peripherals
 *    -      |       -             | of the MCU are always
 *    -      |       -             | operative in this mode
 *  0x01     | POWER_MODE_LOWPOWER | This is first level of power saving mode
 *  0x02     | POWER_MODE_SUSPEND  | In suspend mode the system is
 *   -       |      -              | paused and all the sensors and
 *   -       |      -              | the micro controller are
 *   -       |      -              | put into sleep mode.
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note For detailed about LOWPOWER mode
 *	refer data sheet 3.4.2
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_power_mode(
u8 *v_power_mode_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, power mode is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read the value of power mode */
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_POWER_MODE__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_power_mode_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_POWER_MODE);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!	@brief This API used to write the power mode
 *	from register from 0x3E bit 0 to 1
 *
 *	@param v_power_mode_u8 : The value of power mode
 *
 * v_power_mode_u8 |      result        | comments
 * ----------|---------------------|-------------------------------------
 *  0x00     | POWER_MODE_NORMAL   | In the NORMAL mode the register
 *    -      |       -             | map and the internal peripherals
 *    -      |       -             | of the MCU are always
 *    -      |       -             | operative in this mode
 *  0x01     | POWER_MODE_LOWPOWER | This is first level of power saving mode
 *  0x02     | POWER_MODE_SUSPEND  | In suspend mode the system is
 *   -       |      -              | paused and all the sensors and
 *   -       |      -              | the micro controller are
 *   -       |      -              | put into sleep mode.
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note For detailed about LOWPOWER mode
 *	refer data sheet 3.4.2
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_power_mode(u8 v_power_mode_u8)
{
BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 v_data_u8r = BNO055_ZERO_U8X;
u8 v_prev_opmode_u8 = OPERATION_MODE_CONFIG;
s8 v_stat_s8 = ERROR;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
	} else {
		/* The write operation effective only if the operation
		mode is in config mode, this part of code is checking the
		current operation mode and set the config mode */
		v_stat_s8 = bno055_get_operation_mode(&v_prev_opmode_u8);
		if (v_stat_s8 == SUCCESS) {
			if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
				v_stat_s8 += bno055_set_operation_mode
				(OPERATION_MODE_CONFIG);
			if (v_stat_s8 == SUCCESS) {
				/* Write the value of power mode */
				com_rslt = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_POWER_MODE__REG,
				&v_data_u8r, BNO055_ONE_U8X);
				if (com_rslt == SUCCESS) {
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_POWER_MODE, v_power_mode_u8);
					com_rslt +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_POWER_MODE__REG,
					&v_data_u8r, BNO055_ONE_U8X);
				}
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	}
	if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
		/* set the operation mode
		of previous operation mode*/
		com_rslt += bno055_set_operation_mode
		(v_prev_opmode_u8);
	return com_rslt;
}
/*!
 *	@brief This API used to read the reset interrupt
 *	from register from 0x3F bit 6
 *	It resets all the interrupt bit and interrupt output
 *
 *	@param v_intr_rst_u8 : The value of reset interrupt
 *
 *    v_intr_rst_u8 | result
 *   -------------- |----------
 *     0x01         | ENABLED
 *     0x00         | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_intr_rst(
u8 *v_intr_rst_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page,  reset interrupt is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read the value of reset interrupt*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_INTR_RST__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_intr_rst_u8 =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_INTR_RST);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to write the reset interrupt
 *	from register from 0x3F bit 6
 *	It resets all the interrupt bit and interrupt output
 *
 *	@param v_intr_rst_u8 : The value of reset interrupt
 *
 *    v_intr_rst_u8 | result
 *   -------------- |----------
 *     0x01         | ENABLED
 *     0x00         | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_intr_rst(u8 v_intr_rst_u8)
{
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, reset interrupt
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Write the value of reset interrupt */
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_INTR_RST__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			if (com_rslt == SUCCESS) {
				v_data_u8r =
				BNO055_SET_BITSLICE(v_data_u8r,
				BNO055_INTR_RST, v_intr_rst_u8);
				com_rslt +=
				p_bno055->BNO055_BUS_WRITE_FUNC
				(p_bno055->dev_addr,
				BNO055_INTR_RST__REG,
				&v_data_u8r, BNO055_ONE_U8X);
			}
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to read the clk source
 *	from register from 0x3F bit 7
 *
 *	@param v_clk_src_u8 : The value of clk source
 *
 *	 v_clk_src_u8   | result
 *   -------------- |----------
 *     0x01         | ENABLED
 *     0x00         | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_clk_src(
u8 *v_clk_src_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, clk source is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read the value of clk source */
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_CLK_SRC__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_clk_src_u8 =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_CLK_SRC);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to write the clk source
 *	from register from 0x3F bit 7
 *
 *	@param v_clk_src_u8 : The value of clk source
 *
 *	 v_clk_src_u8   | result
 *   -------------- |----------
 *     0x01         | ENABLED
 *     0x00         | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_clk_src(u8 v_clk_src_u8)
{
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, clk source is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Write the value of clk source */
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_CLK_SRC__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			if (com_rslt == SUCCESS) {
				v_data_u8r =
				BNO055_SET_BITSLICE(v_data_u8r,
				BNO055_CLK_SRC, v_clk_src_u8);
				com_rslt +=
				p_bno055->BNO055_BUS_WRITE_FUNC
				(p_bno055->dev_addr,
				BNO055_CLK_SRC__REG,
				&v_data_u8r, BNO055_ONE_U8X);
			}
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to read the reset system
 *	from register from 0x3F bit 5
 *
 *	@param v_sys_rst_u8 : The value of reset system
 *
 *	 v_sys_rst_u8   | result
 *   -------------- |----------
 *     0x01         | ENABLED
 *     0x00         | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note It resets the whole system
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_sys_rst(
u8 *v_sys_rst_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, reset system is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read the value of reset system */
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_SYS_RST__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_sys_rst_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_SYS_RST);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to write the reset system
 *	from register from 0x3F bit 5
 *
 *	@param v_sys_rst_u8 : The value of reset system
 *
 *	 v_sys_rst_u8   | result
 *   -------------- |----------
 *     0x01         | ENABLED
 *     0x00         | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note It resets the whole system
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_sys_rst(u8 v_sys_rst_u8)
{
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, reset system is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Write the value of reset system */
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_SYS_RST__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			if (com_rslt == SUCCESS) {
				v_data_u8r =
				BNO055_SET_BITSLICE(v_data_u8r,
				BNO055_SYS_RST, v_sys_rst_u8);
				com_rslt =
				p_bno055->BNO055_BUS_WRITE_FUNC
				(p_bno055->dev_addr,
				BNO055_SYS_RST__REG,
				&v_data_u8r, BNO055_ONE_U8X);
			}
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to read the self test
 *	from register from 0x3F bit 0
 *
 *	@param v_selftest_u8 : The value of self test
 *
 *	 v_selftest_u8  | result
 *   -------------- |----------
 *     0x01         | ENABLED
 *     0x00         | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note It triggers the self test
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_selftest(
u8 *v_selftest_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, self test is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read the value of self test */
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_SELFTEST__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_selftest_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_SELFTEST);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to write the self test
 *	from register from 0x3F bit 0
 *
 *	@param v_selftest_u8 : The value of self test
 *
 *	 v_selftest_u8  | result
 *   -------------- |----------
 *     0x01         | ENABLED
 *     0x00         | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note It triggers the self test
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_selftest(u8 v_selftest_u8)
{
BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 v_data_u8r = BNO055_ZERO_U8X;
u8 v_prev_opmode_u8 = OPERATION_MODE_CONFIG;
s8 v_stat_s8 = ERROR;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
	} else {
		/* The write operation effective only if the operation
		mode is in config mode, this part of code is checking the
		current operation mode and set the config mode */
		v_stat_s8 = bno055_get_operation_mode(&v_prev_opmode_u8);
		if (v_stat_s8 == SUCCESS) {
			if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
				v_stat_s8 += bno055_set_operation_mode
				(OPERATION_MODE_CONFIG);
			if (v_stat_s8 == SUCCESS) {
				/* Write the value of self test */
				com_rslt = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_SELFTEST__REG,
				&v_data_u8r, BNO055_ONE_U8X);
				if (com_rslt == SUCCESS) {
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_SELFTEST,
					v_selftest_u8);
					com_rslt +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_SELFTEST__REG,
					&v_data_u8r, BNO055_ONE_U8X);
				}
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	}
	if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
		/* set the operation mode
		of previous operation mode*/
		com_rslt += bno055_set_operation_mode
		(v_prev_opmode_u8);
	return com_rslt;
}
/*!
 *	@brief This API used to read the temperature source
 *	from register from 0x40 bit 0 and 1
 *
 *	@param v_temp_source_u8 : The value of selected temperature source
 *
 *     v_temp_source_u8 | result
 *    ----------------  |---------------
 *      0x00            | ACCEL_TEMP_EN
 *      0X01            | GYRO_TEMP_EN
 *      0X03            | MCU_TEMP_EN
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_temp_source(
u8 *v_temp_source_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, temperature source is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read the value of temperature source */
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_TEMP_SOURCE__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_temp_source_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_TEMP_SOURCE);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to write the temperature source
 *	from register from 0x40 bit 0 and 1
 *
 *	@param v_temp_source_u8 : The value of selected temperature source
 *
 *     v_temp_source_u8 | result
 *    ----------------  |---------------
 *      0x00            | ACCEL_TEMP_EN
 *      0X01            | GYRO_TEMP_EN
 *      0X03            | MCU_TEMP_EN
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_temp_source(u8 v_temp_source_u8)
{
BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 v_data_u8r = BNO055_ZERO_U8X;
u8 v_prev_opmode_u8 = OPERATION_MODE_CONFIG;
s8 v_stat_s8 = ERROR;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
	} else {
		/* The write operation effective only if the operation
		mode is in config mode, this part of code is checking the
		current operation mode and set the config mode */
		v_stat_s8 = bno055_get_operation_mode(&v_prev_opmode_u8);
		if (v_stat_s8 == SUCCESS) {
			if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
				v_stat_s8 += bno055_set_operation_mode
				(OPERATION_MODE_CONFIG);
			if (v_stat_s8 == SUCCESS) {
				/* Write the value of temperature source*/
				com_rslt = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_TEMP_SOURCE__REG,
				&v_data_u8r, BNO055_ONE_U8X);
				if (com_rslt == SUCCESS) {
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_TEMP_SOURCE, v_temp_source_u8);
					com_rslt +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_TEMP_SOURCE__REG,
					&v_data_u8r, BNO055_ONE_U8X);
				}
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	}
	if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
		/* set the operation mode
		of previous operation mode*/
		com_rslt += bno055_set_operation_mode
		(v_prev_opmode_u8);
	return com_rslt;
}
/*!
 *	@brief This API used to read the axis remap value
 *	from register from 0x41 bit 0 and 5
 *
 *	@param v_remap_axis_u8 : The value of axis remapping
 *
 *    v_remap_axis_u8 |   result     | comments
 *   ------------|-------------------|------------
 *      0X21     | REMAP_X_Y         | Z=Z;X=Y;Y=X
 *      0X18     | REMAP_Y_Z         | X=X;Y=Z;Z=Y
 *      0X06     | REMAP_Z_X         | Y=Y;X=Z;Z=X
 *      0X12     | REMAP_X_Y_Z_TYPE0 | X=Z;Y=X;Z=Y
 *      0X09     | REMAP_X_Y_Z_TYPE1 | X=Y;Y=Z;Z=X
 *      0X24     | DEFAULT_AXIS      | X=X;Y=Y;Z=Z
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note : For axis sign remap refer the following functions
 *	x-axis :
 *
 *	bno055_set_x_remap_sign()
 *
 *	y-axis :
 *
 *	bno055_set_y_remap_sign()
 *
 *	z-axis :
 *
 *	bno055_set_z_remap_sign()
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_axis_remap_value(
u8 *v_remap_axis_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, axis remap is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read the value of axis remap*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_REMAP_AXIS_VALUE__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_remap_axis_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_REMAP_AXIS_VALUE);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to write the axis remap value
 *	from register from 0x41 bit 0 and 5
 *
 *	@param v_remap_axis_u8 : The value of axis remapping
 *
 *    v_remap_axis_u8 |   result     | comments
 *   ------------|-------------------|------------
 *      0X21     | REMAP_X_Y         | Z=Z;X=Y;Y=X
 *      0X18     | REMAP_Y_Z         | X=X;Y=Z;Z=Y
 *      0X06     | REMAP_Z_X         | Y=Y;X=Z;Z=X
 *      0X12     | REMAP_X_Y_Z_TYPE0 | X=Z;Y=X;Z=Y
 *      0X09     | REMAP_X_Y_Z_TYPE1 | X=Y;Y=Z;Z=X
 *      0X24     | DEFAULT_AXIS      | X=X;Y=Y;Z=Z
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note : For axis sign remap refer the following functions
 *	x-axis :
 *
 *	bno055_set_x_remap_sign()
 *
 *	y-axis :
 *
 *	bno055_set_y_remap_sign()
 *
 *	z-axis :
 *
 *	bno055_set_z_remap_sign()
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_axis_remap_value(
u8 v_remap_axis_u8)
{
BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 v_data_u8r = BNO055_ZERO_U8X;
u8 v_prev_opmode_u8 = OPERATION_MODE_CONFIG;
s8 v_stat_s8 = ERROR;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
	} else {
	/* The write operation effective only if the operation
	mode is in config mode, this part of code is checking the
	current operation mode and set the config mode */
	v_stat_s8 = bno055_get_operation_mode(&v_prev_opmode_u8);
	if (v_stat_s8 == SUCCESS) {
		if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
			v_stat_s8 += bno055_set_operation_mode
			(OPERATION_MODE_CONFIG);
			/* Write the value of axis remap */
		if (v_stat_s8 == SUCCESS) {
			switch (v_remap_axis_u8) {
			case REMAP_X_Y:
			case REMAP_Y_Z:
			case REMAP_Z_X:
			case REMAP_X_Y_Z_TYPE0:
			case REMAP_X_Y_Z_TYPE1:
			case DEFAULT_AXIS:
				com_rslt =
				p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_REMAP_AXIS_VALUE__REG,
				&v_data_u8r, BNO055_ONE_U8X);
				if (com_rslt == SUCCESS) {
					v_data_u8r = BNO055_SET_BITSLICE
					(v_data_u8r,
					BNO055_REMAP_AXIS_VALUE,
					v_remap_axis_u8);
					com_rslt +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_REMAP_AXIS_VALUE__REG,
					&v_data_u8r, BNO055_ONE_U8X);
				}
			break;
			default:
				/* Write the default axis remap value */
				com_rslt =
				p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_REMAP_AXIS_VALUE__REG,
				&v_data_u8r, BNO055_ONE_U8X);
				if (com_rslt == SUCCESS) {
					v_data_u8r = BNO055_SET_BITSLICE
					(v_data_u8r,
					BNO055_REMAP_AXIS_VALUE,
					DEFAULT_AXIS);
					com_rslt +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_REMAP_AXIS_VALUE__REG,
					&v_data_u8r, BNO055_ONE_U8X);
				}
			break;
			}
		} else {
		com_rslt = ERROR;
		}
	} else {
	com_rslt = ERROR;
	}
}
if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
	/* set the operation mode
	of previous operation mode*/
	com_rslt += bno055_set_operation_mode
	(v_prev_opmode_u8);
return com_rslt;
}
/*!
 *	@brief This API used to read the x-axis remap
 *	sign from register from 0x42 bit 2
 *
 *	@param v_remap_x_sign_u8 : The value of x-axis remap sign
 *
 *    v_remap_x_sign_u8  |    result
 *   ------------------- |--------------------
 *      0X00             | REMAP_AXIS_POSITIVE
 *      0X01             | REMAP_AXIS_NEGATIVE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_remap_x_sign(
u8 *v_remap_x_sign_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, x-axis remap sign is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read the value of x-axis remap sign */
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_REMAP_X_SIGN__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_remap_x_sign_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_REMAP_X_SIGN);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to write the x-axis remap
 *	sign from register from 0x42 bit 2
 *
 *	@param v_remap_x_sign_u8 : The value of x-axis remap sign
 *
 *    v_remap_x_sign_u8  |    result
 *   ------------------- |--------------------
 *      0X00             | REMAP_AXIS_POSITIVE
 *      0X01             | REMAP_AXIS_NEGATIVE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_remap_x_sign(
u8 v_remap_x_sign_u8)
{
BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 v_data_u8r = BNO055_ZERO_U8X;
u8 v_prev_opmode_u8 = OPERATION_MODE_CONFIG;
s8 v_stat_s8 = ERROR;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
	} else {
		/* The write operation effective only if the operation
		mode is in config mode, this part of code is checking the
		current operation mode and set the config mode */
		v_stat_s8 = bno055_get_operation_mode(&v_prev_opmode_u8);
		if (v_stat_s8 == SUCCESS) {
			if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
				v_stat_s8 += bno055_set_operation_mode
				(OPERATION_MODE_CONFIG);
			if (v_stat_s8 == SUCCESS) {
				/* Write the value of x-axis remap */
				com_rslt = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_REMAP_X_SIGN__REG,
				&v_data_u8r, BNO055_ONE_U8X);
				if (com_rslt == SUCCESS) {
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_REMAP_X_SIGN,
					v_remap_x_sign_u8);
					com_rslt +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_REMAP_X_SIGN__REG,
					&v_data_u8r, BNO055_ONE_U8X);
				}
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	}
	if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
		/* set the operation mode
		of previous operation mode*/
		com_rslt += bno055_set_operation_mode
		(v_prev_opmode_u8);
	return com_rslt;
}
/*!
 *	@brief This API used to read the y-axis remap
 *	sign from register from 0x42 bit 1
 *
 *	@param v_remap_y_sign_u8 : The value of y-axis remap sign
 *
 *    v_remap_y_sign_u8  |   result
 *   ------------------- |--------------------
 *      0X00             | REMAP_AXIS_POSITIVE
 *      0X01             | REMAP_AXIS_NEGATIVE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_remap_y_sign(
u8 *v_remap_y_sign_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, y-axis remap sign is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read the value of y-axis remap sign*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_REMAP_Y_SIGN__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_remap_y_sign_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_REMAP_Y_SIGN);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to write the y-axis remap
 *	sign from register from 0x42 bit 1
 *
 *	@param v_remap_y_sign_u8 : The value of y-axis remap sign
 *
 *    v_remap_y_sign_u8  |   result
 *   ------------------- |--------------------
 *      0X00             | REMAP_AXIS_POSITIVE
 *      0X01             | REMAP_AXIS_NEGATIVE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_remap_y_sign(
u8 v_remap_y_sign_u8)
{
BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 v_data_u8r = BNO055_ZERO_U8X;
u8 v_prev_opmode_u8 = OPERATION_MODE_CONFIG;
s8 v_stat_s8 = ERROR;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
	} else {
		/* The write operation effective only if the operation
		mode is in config mode, this part of code is checking the
		current operation mode and set the config mode */
		v_stat_s8 = bno055_get_operation_mode(&v_prev_opmode_u8);
		if (v_stat_s8 == SUCCESS) {
			if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
				v_stat_s8 += bno055_set_operation_mode
				(OPERATION_MODE_CONFIG);
			if (v_stat_s8 == SUCCESS) {
				/* Write the value of y-axis remap sign*/
				com_rslt = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_REMAP_Y_SIGN__REG,
				&v_data_u8r, BNO055_ONE_U8X);
				if (com_rslt == SUCCESS) {
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_REMAP_Y_SIGN,
					v_remap_y_sign_u8);
					com_rslt +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_REMAP_Y_SIGN__REG,
					&v_data_u8r, BNO055_ONE_U8X);
				}
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	}
	if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
		/* set the operation mode
		of previous operation mode*/
		com_rslt += bno055_set_operation_mode
		(v_prev_opmode_u8);
	return com_rslt;
}
/*!
 *	@brief This API used to read the z-axis remap
 *	sign from register from 0x42 bit 0
 *
 *	@param v_remap_z_sign_u8 : The value of z-axis remap sign
 *
 *    v_remap_z_sign_u8  |   result
 *   ------------------- |--------------------
 *      0X00             | REMAP_AXIS_POSITIVE
 *      0X01             | REMAP_AXIS_NEGATIVE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_remap_z_sign(
u8 *v_remap_z_sign_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, z-axis remap sign is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read the value of z-axis remap sign*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_REMAP_Z_SIGN__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_remap_z_sign_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_REMAP_Z_SIGN);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to write the z-axis remap
 *	sign from register from 0x42 bit 0
 *
 *	@param v_remap_z_sign_u8 : The value of z-axis remap sign
 *
 *    v_remap_z_sign_u8  |   result
 *   ------------------- |--------------------
 *      0X00             | REMAP_AXIS_POSITIVE
 *      0X01             | REMAP_AXIS_NEGATIVE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_remap_z_sign(
u8 v_remap_z_sign_u8)
{
BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 v_data_u8r = BNO055_ZERO_U8X;
u8 v_prev_opmode_u8 = OPERATION_MODE_CONFIG;
s8 v_stat_s8 = ERROR;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
	} else {
		/* The write operation effective only if the operation
		mode is in config mode, this part of code is checking the
		current operation mode and set the config mode */
		v_stat_s8 = bno055_get_operation_mode(&v_prev_opmode_u8);
		if (v_stat_s8 == SUCCESS) {
			if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
				v_stat_s8 += bno055_set_operation_mode
				(OPERATION_MODE_CONFIG);
			if (v_stat_s8 == SUCCESS) {
				/* Write the value of z-axis remap sign*/
				com_rslt = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_REMAP_Z_SIGN__REG,
				&v_data_u8r, BNO055_ONE_U8X);
				if (com_rslt == SUCCESS) {
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_REMAP_Z_SIGN,
					v_remap_z_sign_u8);
					com_rslt +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_REMAP_Z_SIGN__REG,
					&v_data_u8r, BNO055_ONE_U8X);
				}
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	}
	if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
		/* set the operation mode
		of previous operation mode*/
		com_rslt += bno055_set_operation_mode
		(v_prev_opmode_u8);
	return com_rslt;
}
/*!
 *	@brief This API is used to read soft iron calibration matrix
 *	from the register 0x43 to 0x53 it is a 18 bytes of data
 *
 *	@param sic_matrix : The value of soft iron calibration matrix
 *
 *	sic_matrix         |           result
 * --------------------|----------------------------------
 *       sic_0         | soft iron calibration matrix zero
 *       sic_1         | soft iron calibration matrix one
 *       sic_2         | soft iron calibration matrix two
 *       sic_3         | soft iron calibration matrix three
 *       sic_4         | soft iron calibration matrix four
 *       sic_5         | soft iron calibration matrix five
 *       sic_6         | soft iron calibration matrix six
 *       sic_7         | soft iron calibration matrix seven
 *       sic_8         | soft iron calibration matrix eight
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note : Each soft iron calibration matrix range from -32768 to +32767
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_sic_matrix(
struct bno055_sic_matrix_t  *sic_matrix)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	/* Array holding the soft iron calibration matrix values
	v_data_u8[INDEX_ZERO] - sic_0->LSB
	v_data_u8[INDEX_ONE] - sic_0->MSB
	v_data_u8[INDEX_TWO] - sic_1->LSB
	v_data_u8[INDEX_THREE] - sic_1->MSB
	v_data_u8[INDEX_FOUR] - sic_2->LSB
	v_data_u8[INDEX_FIVE] - sic_2->MSB
	v_data_u8[6] - sic_3->LSB
	v_data_u8[7] - sic_3->MSB
	v_data_u8[8] - sic_4->LSB
	v_data_u8[9] - sic_4->MSB
	v_data_u8[10] - sic_5->LSB
	v_data_u8[11] - sic_5->MSB
	v_data_u8[12] - sic_6->LSB
	v_data_u8[13] - sic_6->MSB
	v_data_u8[14] - sic_7->LSB
	v_data_u8[15] - sic_7->MSB
	v_data_u8[16] - sic_8->LSB
	v_data_u8[17] - sic_8->MSB
	*/
	u8 v_data_u8[ARRAY_SIZE_EIGHTEEN] = {
	BNO055_ZERO_U8X, BNO055_ZERO_U8X,
	BNO055_ZERO_U8X, BNO055_ZERO_U8X,
	BNO055_ZERO_U8X, BNO055_ZERO_U8X,
	BNO055_ZERO_U8X, BNO055_ZERO_U8X,
	BNO055_ZERO_U8X, BNO055_ZERO_U8X,
	BNO055_ZERO_U8X, BNO055_ZERO_U8X,
	BNO055_ZERO_U8X, BNO055_ZERO_U8X,
	BNO055_ZERO_U8X, BNO055_ZERO_U8X,
	BNO055_ZERO_U8X, BNO055_ZERO_U8X};
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, soft iron calibration matrix is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read soft iron calibration matrix value
			it is eighteen bytes of data */
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_SIC_MATRIX_0_LSB__REG,
			v_data_u8, BNO055_EIGHTEEN_U8X);
			if (com_rslt == SUCCESS) {
				/*soft iron calibration matrix zero*/
				v_data_u8[INDEX_ZERO] =
				BNO055_GET_BITSLICE(v_data_u8[INDEX_ZERO],
				BNO055_SIC_MATRIX_0_LSB);
				v_data_u8[INDEX_ONE] =
				BNO055_GET_BITSLICE(v_data_u8[INDEX_ONE],
				BNO055_SIC_MATRIX_0_MSB);
				sic_matrix->sic_0 = (s16)((((s32)
				(s8)(v_data_u8[INDEX_ONE])) <<
				(BNO055_SHIFT_8_POSITION))
				| (v_data_u8[INDEX_ZERO]));

				/*soft iron calibration matrix one*/
				v_data_u8[INDEX_TWO] =
				BNO055_GET_BITSLICE(v_data_u8[INDEX_TWO],
				BNO055_SIC_MATRIX_1_LSB);
				v_data_u8[INDEX_THREE] =
				BNO055_GET_BITSLICE(v_data_u8[INDEX_THREE],
				BNO055_SIC_MATRIX_1_MSB);
				sic_matrix->sic_1 = (s16)((((s32)
				(s8)(v_data_u8[INDEX_THREE])) <<
				(BNO055_SHIFT_8_POSITION))
				| (v_data_u8[INDEX_TWO]));

				/*soft iron calibration matrix two*/
				v_data_u8[INDEX_FOUR] =
				BNO055_GET_BITSLICE(v_data_u8[INDEX_FOUR],
				BNO055_SIC_MATRIX_2_LSB);
				v_data_u8[INDEX_FIVE] =
				BNO055_GET_BITSLICE(v_data_u8[INDEX_FIVE],
				BNO055_SIC_MATRIX_2_MSB);
				sic_matrix->sic_2 = (s16)((((s32)
				(s8)(v_data_u8[INDEX_FIVE])) <<
				(BNO055_SHIFT_8_POSITION))
				| (v_data_u8[INDEX_FOUR]));

				/*soft iron calibration matrix three*/
				v_data_u8[INDEX_SIX] =
				BNO055_GET_BITSLICE(v_data_u8[INDEX_SIX],
				BNO055_SIC_MATRIX_3_LSB);
				v_data_u8[INDEX_SEVEN] =
				BNO055_GET_BITSLICE(v_data_u8[INDEX_SEVEN],
				BNO055_SIC_MATRIX_3_LSB);
				sic_matrix->sic_3  = (s16)((((s32)
				(s8)(v_data_u8[INDEX_SEVEN])) <<
				(BNO055_SHIFT_8_POSITION)) |
				(v_data_u8[INDEX_SIX]));

				/*soft iron calibration matrix four*/
				v_data_u8[INDEX_EIGHT] =
				BNO055_GET_BITSLICE(v_data_u8[INDEX_EIGHT],
				BNO055_SIC_MATRIX_4_LSB);
				v_data_u8[INDEX_NINE] =
				BNO055_GET_BITSLICE(v_data_u8[INDEX_NINE],
				BNO055_SIC_MATRIX_4_LSB);
				sic_matrix->sic_4  = (s16)((((s32)
				(s8)(v_data_u8[INDEX_NINE])) <<
				(BNO055_SHIFT_8_POSITION)) |
				(v_data_u8[INDEX_EIGHT]));

				/*soft iron calibration matrix five*/
				v_data_u8[INDEX_TEN] =
				BNO055_GET_BITSLICE(v_data_u8[INDEX_TEN],
				BNO055_SIC_MATRIX_5_LSB);
				v_data_u8[INDEX_ELEVEN] =
				BNO055_GET_BITSLICE(v_data_u8[INDEX_ELEVEN],
				BNO055_SIC_MATRIX_5_LSB);
				sic_matrix->sic_5 = (s16)((((s32)
				(s8)(v_data_u8[INDEX_ELEVEN])) <<
				(BNO055_SHIFT_8_POSITION)) |
				(v_data_u8[INDEX_TEN]));

				/*soft iron calibration matrix six*/
				v_data_u8[INDEX_TWELVE] =
				BNO055_GET_BITSLICE(v_data_u8[INDEX_TWELVE],
				BNO055_SIC_MATRIX_6_LSB);
				v_data_u8[INDEX_THIRTEEN] =
				BNO055_GET_BITSLICE(v_data_u8[INDEX_THIRTEEN],
				BNO055_SIC_MATRIX_6_LSB);
				sic_matrix->sic_6  = (s16)((((s32)
				(s8)(v_data_u8[INDEX_THIRTEEN])) <<
				(BNO055_SHIFT_8_POSITION)) |
				(v_data_u8[INDEX_TWELVE]));

				/*soft iron calibration matrix seven*/
				v_data_u8[INDEX_FOURTEEN] =
				BNO055_GET_BITSLICE(v_data_u8[INDEX_FOURTEEN],
				BNO055_SIC_MATRIX_7_LSB);
				v_data_u8[INDEX_FIVETEEN] =
				BNO055_GET_BITSLICE(v_data_u8[INDEX_FIVETEEN],
				BNO055_SIC_MATRIX_7_LSB);
				sic_matrix->sic_7  = (s16)((((s32)
				(s8)(v_data_u8[INDEX_FIVETEEN])) <<
				(BNO055_SHIFT_8_POSITION)) |
				(v_data_u8[INDEX_FOURTEEN]));

				/*soft iron calibration matrix eight*/
				v_data_u8[INDEX_SIXTEEN] =
				BNO055_GET_BITSLICE(v_data_u8[INDEX_SIXTEEN],
				BNO055_SIC_MATRIX_8_LSB);
				v_data_u8[INDEX_SEVENTEEN] =
				BNO055_GET_BITSLICE(v_data_u8[INDEX_SEVENTEEN],
				BNO055_SIC_MATRIX_8_LSB);
				sic_matrix->sic_8  = (s16)((((s32)
				(s8)(v_data_u8[INDEX_SEVENTEEN])) <<
				(BNO055_SHIFT_8_POSITION)) |
				(v_data_u8[INDEX_SIXTEEN]));
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to write soft iron calibration matrix
 *	from the register 0x43 to 0x53 it is a 18 bytes of data
 *
 *	@param sic_matrix : The value of soft iron calibration matrix
 *
 *	sic_matrix         |           result
 * --------------------|----------------------------------
 *       sic_0         | soft iron calibration matrix zero
 *       sic_1         | soft iron calibration matrix one
 *       sic_2         | soft iron calibration matrix two
 *       sic_3         | soft iron calibration matrix three
 *       sic_4         | soft iron calibration matrix four
 *       sic_5         | soft iron calibration matrix five
 *       sic_6         | soft iron calibration matrix six
 *       sic_7         | soft iron calibration matrix seven
 *       sic_8         | soft iron calibration matrix eight
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note : Each soft iron calibration matrix range from -32768 to +32767
 */
BNO055_RETURN_FUNCTION_TYPE bno055_write_sic_matrix(
struct bno055_sic_matrix_t  *sic_matrix)
{
BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 v_data1_u8r = BNO055_ZERO_U8X;
u8 v_data2_u8r = BNO055_ZERO_U8X;
u8 v_prev_opmode_u8 = OPERATION_MODE_CONFIG;
s8 v_stat_s8 = ERROR;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
	} else {
	/* The write operation effective only if the operation
		mode is in config mode, this part of code is checking the
		current operation mode and set the config mode */
		v_stat_s8 = bno055_get_operation_mode(&v_prev_opmode_u8);
		if (v_stat_s8 == SUCCESS) {
			if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
				v_stat_s8 += bno055_set_operation_mode
				(OPERATION_MODE_CONFIG);
				if (v_stat_s8 == SUCCESS) {
					/* write soft iron calibration
					matrix zero value*/
					com_rslt =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_0_LSB__REG,
					&v_data2_u8r, BNO055_ONE_U8X);
					if (com_rslt == SUCCESS) {
						v_data1_u8r = ((s8)
						(sic_matrix->sic_0
						& BNO055_SIC_HEX_0_0_F_F_DATA));
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_SIC_MATRIX_0_LSB,
						v_data1_u8r);
						com_rslt +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_SIC_MATRIX_0_LSB__REG,
						&v_data2_u8r, BNO055_ONE_U8X);
					}

					com_rslt +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_0_MSB__REG,
					&v_data2_u8r, BNO055_ONE_U8X);
					if (com_rslt == SUCCESS) {
						v_data1_u8r = ((s8)
						(sic_matrix->sic_0  >>
						BNO055_SHIFT_8_POSITION)
						& BNO055_SIC_HEX_0_0_F_F_DATA);
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_SIC_MATRIX_0_MSB,
						v_data1_u8r);
						com_rslt +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_SIC_MATRIX_0_MSB__REG,
						&v_data2_u8r, BNO055_ONE_U8X);
					}

					/* write soft iron calibration
					matrix one value*/
					com_rslt +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_1_LSB__REG,
					&v_data2_u8r, BNO055_ONE_U8X);
					if (com_rslt == SUCCESS) {
						v_data1_u8r = ((s8)
						(sic_matrix->sic_1
						& BNO055_SIC_HEX_0_0_F_F_DATA));
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_SIC_MATRIX_1_LSB,
						v_data1_u8r);
						com_rslt +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_SIC_MATRIX_1_LSB__REG,
						&v_data2_u8r, BNO055_ONE_U8X);
					}

					com_rslt +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_1_MSB__REG,
					&v_data2_u8r, BNO055_ONE_U8X);
					if (com_rslt == SUCCESS) {
						v_data1_u8r = ((s8)
						(sic_matrix->sic_1  >>
						BNO055_SHIFT_8_POSITION)
						& BNO055_SIC_HEX_0_0_F_F_DATA);
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_SIC_MATRIX_1_MSB,
						v_data1_u8r);
						com_rslt +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_SIC_MATRIX_1_MSB__REG,
						&v_data2_u8r, BNO055_ONE_U8X);
					}

				/* write soft iron calibration
				matrix two value*/
					com_rslt +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_2_LSB__REG,
					&v_data2_u8r, BNO055_ONE_U8X);
					if (com_rslt == SUCCESS) {
						v_data1_u8r = ((s8)
						(sic_matrix->sic_2
						& BNO055_SIC_HEX_0_0_F_F_DATA));
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_SIC_MATRIX_2_LSB,
						v_data1_u8r);
						com_rslt +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_SIC_MATRIX_2_LSB__REG,
						&v_data2_u8r, BNO055_ONE_U8X);
					}

					com_rslt +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_2_MSB__REG,
					&v_data2_u8r, BNO055_ONE_U8X);
					if (com_rslt == SUCCESS) {
						v_data1_u8r = ((s8)
						(sic_matrix->sic_2 >>
						BNO055_SHIFT_8_POSITION)
						& BNO055_SIC_HEX_0_0_F_F_DATA);
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_SIC_MATRIX_2_MSB,
						v_data1_u8r);
						com_rslt +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_SIC_MATRIX_2_MSB__REG,
						&v_data2_u8r, BNO055_ONE_U8X);
					}

					/* write soft iron calibration
					matrix three value*/
					com_rslt +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_3_LSB__REG,
					&v_data2_u8r, BNO055_ONE_U8X);
					if (com_rslt == SUCCESS) {
						v_data1_u8r = ((s8)
						(sic_matrix->sic_3
						& BNO055_SIC_HEX_0_0_F_F_DATA));
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_SIC_MATRIX_3_LSB,
						v_data1_u8r);
						com_rslt +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_SIC_MATRIX_3_LSB__REG,
						&v_data2_u8r, BNO055_ONE_U8X);
					}

					com_rslt +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_3_MSB__REG,
					&v_data2_u8r, BNO055_ONE_U8X);
					if (com_rslt == SUCCESS) {
						v_data1_u8r = ((s8)
						(sic_matrix->sic_3 >>
						BNO055_SHIFT_8_POSITION)
						& BNO055_SIC_HEX_0_0_F_F_DATA);
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_SIC_MATRIX_3_MSB,
						v_data1_u8r);
						com_rslt +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_SIC_MATRIX_3_MSB__REG,
						&v_data2_u8r, BNO055_ONE_U8X);
					}

					/* write soft iron calibration
					matrix four value*/
					com_rslt +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_4_LSB__REG,
					&v_data2_u8r, BNO055_ONE_U8X);
					if (com_rslt == SUCCESS) {
						v_data1_u8r = ((s8)
						(sic_matrix->sic_4
						& BNO055_SIC_HEX_0_0_F_F_DATA));
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_SIC_MATRIX_4_LSB,
						v_data1_u8r);
						com_rslt +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_SIC_MATRIX_4_LSB__REG,
						&v_data2_u8r, BNO055_ONE_U8X);
					}

					com_rslt +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_4_MSB__REG,
					&v_data2_u8r, BNO055_ONE_U8X);
					if (com_rslt == SUCCESS) {
						v_data1_u8r = ((s8)
						(sic_matrix->sic_4 >>
						BNO055_SHIFT_8_POSITION)
						& BNO055_SIC_HEX_0_0_F_F_DATA);
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_SIC_MATRIX_4_MSB,
						v_data1_u8r);
						com_rslt +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_SIC_MATRIX_4_MSB__REG,
						&v_data2_u8r, BNO055_ONE_U8X);
					}

					/* write soft iron calibration
					matrix five value*/
					com_rslt +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_5_LSB__REG,
					&v_data2_u8r, BNO055_ONE_U8X);
					if (com_rslt == SUCCESS) {
						v_data1_u8r = ((s8)
						(sic_matrix->sic_5
						& BNO055_SIC_HEX_0_0_F_F_DATA));
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_SIC_MATRIX_5_LSB,
						v_data1_u8r);
						com_rslt +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_SIC_MATRIX_5_LSB__REG,
						&v_data2_u8r, BNO055_ONE_U8X);
					}

					com_rslt +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_5_MSB__REG,
					&v_data2_u8r, BNO055_ONE_U8X);
					if (com_rslt == SUCCESS) {
						v_data1_u8r = ((s8)
						(sic_matrix->sic_5 >>
						BNO055_SHIFT_8_POSITION)
						& BNO055_SIC_HEX_0_0_F_F_DATA);
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_SIC_MATRIX_5_MSB,
						v_data1_u8r);
						com_rslt +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_SIC_MATRIX_5_MSB__REG,
						&v_data2_u8r, BNO055_ONE_U8X);
					}

					/* write soft iron calibration
					matrix six value*/
					com_rslt +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_6_LSB__REG,
					&v_data2_u8r, BNO055_ONE_U8X);
					if (com_rslt == SUCCESS) {
						v_data1_u8r = ((s8)
						(sic_matrix->sic_6
						& BNO055_SIC_HEX_0_0_F_F_DATA));
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_SIC_MATRIX_6_LSB,
						v_data1_u8r);
						com_rslt +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_SIC_MATRIX_6_LSB__REG,
						&v_data2_u8r, BNO055_ONE_U8X);
					}

					com_rslt +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_6_MSB__REG,
					&v_data2_u8r, BNO055_ONE_U8X);
					if (com_rslt == SUCCESS) {
						v_data1_u8r = ((s8)
						(sic_matrix->sic_6 >>
						BNO055_SHIFT_8_POSITION)
						& BNO055_SIC_HEX_0_0_F_F_DATA);
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_SIC_MATRIX_6_MSB,
						v_data1_u8r);
						com_rslt +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_SIC_MATRIX_6_MSB__REG,
						&v_data2_u8r, BNO055_ONE_U8X);
					}

					/* write soft iron calibration
					matrix seven value*/
					com_rslt +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_7_LSB__REG,
					&v_data2_u8r, BNO055_ONE_U8X);
					if (com_rslt == SUCCESS) {
						v_data1_u8r = ((s8)
						(sic_matrix->sic_7
						& BNO055_SIC_HEX_0_0_F_F_DATA));
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_SIC_MATRIX_7_LSB,
						v_data1_u8r);
						com_rslt +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_SIC_MATRIX_7_LSB__REG,
						&v_data2_u8r, BNO055_ONE_U8X);
					}

					com_rslt +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_7_MSB__REG,
					&v_data2_u8r, BNO055_ONE_U8X);
					if (com_rslt == SUCCESS) {
						v_data1_u8r = ((s8)
						(sic_matrix->sic_7 >>
						BNO055_SHIFT_8_POSITION)
						& BNO055_SIC_HEX_0_0_F_F_DATA);
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_SIC_MATRIX_7_MSB,
						v_data1_u8r);
						com_rslt +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_SIC_MATRIX_7_MSB__REG,
						&v_data2_u8r, BNO055_ONE_U8X);
					}

					/* write soft iron calibration
					matrix eight value*/
					com_rslt +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_8_LSB__REG,
					&v_data2_u8r, BNO055_ONE_U8X);
					if (com_rslt == SUCCESS) {
						v_data1_u8r = ((s8)
						(sic_matrix->sic_8
						& BNO055_SIC_HEX_0_0_F_F_DATA));
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_SIC_MATRIX_8_LSB,
						v_data1_u8r);
						com_rslt +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_SIC_MATRIX_8_LSB__REG,
						&v_data2_u8r, BNO055_ONE_U8X);
					}

					com_rslt +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_8_MSB__REG,
					&v_data2_u8r, BNO055_ONE_U8X);
					if (com_rslt == SUCCESS) {
						v_data1_u8r = ((s8)
						(sic_matrix->sic_8 >>
						BNO055_SHIFT_8_POSITION)
						& BNO055_SIC_HEX_0_0_F_F_DATA);
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_SIC_MATRIX_8_MSB,
						v_data1_u8r);
						com_rslt +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_SIC_MATRIX_8_MSB__REG,
						&v_data2_u8r, BNO055_ONE_U8X);
					}
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	}
	if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
		/* set the operation mode
		of previous operation mode*/
		com_rslt += bno055_set_operation_mode
		(v_prev_opmode_u8);
	return com_rslt;
}
/*!
 *	@brief This API is used to read accel offset and accel radius
 *	offset form register 0x55 to 0x5A and radius form 0x67 and 0x68
 *
 *	@param accel_offset : The value of accel offset and radius
 *
 *	    bno055_accel_offset_t |     result
 *      ------------------- | ----------------
 *	             x          |  accel offset x
 *               y          |  accel offset y
 *	             z          |  accel offset z
 *               r          |  accel offset r
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note  The range of the accel offset varies based on
 *	the G-range of accel sensor.
 *
 *  accel G range   |  offset range
 * ---------------  |  --------------
 *  ACCEL_RANGE_2G  |   +/-2000
 *  ACCEL_RANGE_4G  |   +/-4000
 *  ACCEL_RANGE_8G  |   +/-8000
 *  ACCEL_RANGE_16G |   +/-16000
 *
 *	accel G range can be configured by using the
 *	bno055_set_accel_range() function
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_accel_offset(
struct bno055_accel_offset_t  *accel_offset)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	/* Array holding the accel offset values
	v_data_u8[INDEX_ZERO] - offset x->LSB
	v_data_u8[INDEX_ONE] - offset x->MSB
	v_data_u8[INDEX_TWO] - offset y->LSB
	v_data_u8[INDEX_THREE] - offset y->MSB
	v_data_u8[INDEX_FOUR] - offset z->LSB
	v_data_u8[INDEX_FIVE] - offset z->MSB
	*/
	u8 v_data_u8[ARRAY_SIZE_SIX] = {
	BNO055_ZERO_U8X, BNO055_ZERO_U8X,
	BNO055_ZERO_U8X, BNO055_ZERO_U8X,
	BNO055_ZERO_U8X, BNO055_ZERO_U8X};
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel offset is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read accel offset value it is six bytes of data*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_OFFSET_X_LSB__REG,
			v_data_u8, BNO055_SIX_U8X);
			if (com_rslt == SUCCESS) {
				/* Read accel x offset value*/
				v_data_u8[INDEX_ZERO] =
				BNO055_GET_BITSLICE(v_data_u8[INDEX_ZERO],
				BNO055_ACCEL_OFFSET_X_LSB);
				v_data_u8[INDEX_ONE] =
				BNO055_GET_BITSLICE(v_data_u8[INDEX_ONE],
				BNO055_ACCEL_OFFSET_X_MSB);
				accel_offset->x = (s16)((((s32)
				(s8)(v_data_u8[INDEX_ONE])) <<
				(BNO055_SHIFT_8_POSITION)) |
				(v_data_u8[INDEX_ZERO]));

				/* Read accel y offset value*/
				v_data_u8[INDEX_TWO] =
				BNO055_GET_BITSLICE(v_data_u8[INDEX_TWO],
				BNO055_ACCEL_OFFSET_Y_LSB);
				v_data_u8[INDEX_THREE] =
				BNO055_GET_BITSLICE(v_data_u8[INDEX_THREE],
				BNO055_ACCEL_OFFSET_Y_MSB);
				accel_offset->y = (s16)((((s32)
				(s8)(v_data_u8[INDEX_THREE])) <<
				(BNO055_SHIFT_8_POSITION))
				| (v_data_u8[INDEX_TWO]));

				/* Read accel z offset value*/
				v_data_u8[INDEX_FOUR] =
				BNO055_GET_BITSLICE(v_data_u8[INDEX_FOUR],
				BNO055_ACCEL_OFFSET_Z_LSB);
				v_data_u8[INDEX_FIVE] =
				BNO055_GET_BITSLICE(v_data_u8[INDEX_FIVE],
				BNO055_ACCEL_OFFSET_Z_MSB);
				accel_offset->z = (s16)((((s32)
				(s8)(v_data_u8[INDEX_FIVE])) <<
				(BNO055_SHIFT_8_POSITION)) |
				(v_data_u8[INDEX_FOUR]));

				/* Read accel radius value
				it is two bytes of data*/
				com_rslt += p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_ACCEL_RADIUS_LSB__REG,
				v_data_u8, BNO055_TWO_U8X);
				/* Array holding the accel radius values
				v_data_u8[INDEX_ZERO] - radius->LSB
				v_data_u8[INDEX_ONE] - radius->MSB
				*/
				if (com_rslt == SUCCESS) {
					v_data_u8[INDEX_ZERO] =
					BNO055_GET_BITSLICE(
						v_data_u8[INDEX_ZERO],
					BNO055_ACCEL_RADIUS_LSB);
					v_data_u8[INDEX_ONE] =
					BNO055_GET_BITSLICE(
						v_data_u8[INDEX_ONE],
					BNO055_ACCEL_RADIUS_MSB);
					accel_offset->r = (s16)((((s32)
					(s8)(v_data_u8[INDEX_ONE])) <<
					(BNO055_SHIFT_8_POSITION)) |
					(v_data_u8[INDEX_ZERO]));
				} else {
				com_rslt = ERROR;
				}
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to write accel offset and accel radius
 *	offset form register 0x55 to 0x5A and radius form 0x67 and 0x68
 *
 *	@param accel_offset : The value of accel offset and radius
 *
 *	    bno055_accel_offset_t |     result
 *      ------------------- | ----------------
 *	             x          |  accel offset x
 *               y          |  accel offset y
 *	             z          |  accel offset z
 *               r          |  accel offset r
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note  The range of the accel offset varies based on
 *	the G-range of accel sensor.
 *
 *  accel G range   |  offset range
 * ---------------  |  --------------
 *  ACCEL_RANGE_2G  |   +/-2000
 *  ACCEL_RANGE_4G  |   +/-4000
 *  ACCEL_RANGE_8G  |   +/-8000
 *  ACCEL_RANGE_16G |   +/-16000
 *
 *	accel G range can be configured by using the
 *	bno055_set_accel_range() function
 */
BNO055_RETURN_FUNCTION_TYPE bno055_write_accel_offset(
struct bno055_accel_offset_t  *accel_offset)
{
BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 v_data1_u8r = BNO055_ZERO_U8X;
u8 v_data2_u8r = BNO055_ZERO_U8X;
u8 v_prev_opmode_u8 = OPERATION_MODE_CONFIG;
s8 v_stat_s8 = ERROR;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
	} else {
	/* The write operation effective only if the operation
		mode is in config mode, this part of code is checking the
		current operation mode and set the config mode */
		v_stat_s8 = bno055_get_operation_mode(&v_prev_opmode_u8);
		if (v_stat_s8 == SUCCESS) {
			if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
				v_stat_s8 += bno055_set_operation_mode
				(OPERATION_MODE_CONFIG);
				if (v_stat_s8 == SUCCESS) {
					/* write accel offset x value*/
					com_rslt =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_ACCEL_OFFSET_X_LSB__REG,
					&v_data2_u8r, BNO055_ONE_U8X);
					if (com_rslt == SUCCESS) {
						v_data1_u8r = ((s8)
						(accel_offset->x
						& BNO055_SIC_HEX_0_0_F_F_DATA));
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_ACCEL_OFFSET_X_LSB,
						v_data1_u8r);
						com_rslt +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_ACCEL_OFFSET_X_LSB__REG,
						&v_data2_u8r, BNO055_ONE_U8X);
					}

					com_rslt +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_ACCEL_OFFSET_X_MSB__REG,
					&v_data2_u8r, BNO055_ONE_U8X);
					if (com_rslt == SUCCESS) {
						v_data1_u8r = ((s8)
						(accel_offset->x  >>
						BNO055_SHIFT_8_POSITION)
						& BNO055_SIC_HEX_0_0_F_F_DATA);
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_ACCEL_OFFSET_X_MSB,
						v_data1_u8r);
						com_rslt +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_ACCEL_OFFSET_X_MSB__REG,
						&v_data2_u8r, BNO055_ONE_U8X);
					}

					/* write accel offset y value*/
					com_rslt +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_ACCEL_OFFSET_Y_LSB__REG,
					&v_data2_u8r, BNO055_ONE_U8X);
					if (com_rslt == SUCCESS) {
						v_data1_u8r = ((s8)
						(accel_offset->y
						& BNO055_SIC_HEX_0_0_F_F_DATA));
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_ACCEL_OFFSET_Y_LSB,
						v_data1_u8r);
						com_rslt +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_ACCEL_OFFSET_Y_LSB__REG,
						&v_data2_u8r, BNO055_ONE_U8X);
					}

					com_rslt +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_ACCEL_OFFSET_Y_MSB__REG,
					&v_data2_u8r, BNO055_ONE_U8X);
					if (com_rslt == SUCCESS) {
						v_data1_u8r = ((s8)
						(accel_offset->y >>
						BNO055_SHIFT_8_POSITION)
						& BNO055_SIC_HEX_0_0_F_F_DATA);
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_ACCEL_OFFSET_Y_MSB,
						v_data1_u8r);
						com_rslt +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_ACCEL_OFFSET_Y_MSB__REG,
						&v_data2_u8r, BNO055_ONE_U8X);
					}
				/* write accel offset z value*/
					com_rslt +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_ACCEL_OFFSET_Z_LSB__REG,
					&v_data2_u8r, BNO055_ONE_U8X);
					if (com_rslt == SUCCESS) {
						v_data1_u8r = ((s8)
						(accel_offset->z
						& BNO055_SIC_HEX_0_0_F_F_DATA));
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_ACCEL_OFFSET_Z_LSB,
						v_data1_u8r);
						com_rslt +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_ACCEL_OFFSET_Z_LSB__REG,
						&v_data2_u8r, BNO055_ONE_U8X);
					}

					com_rslt +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_ACCEL_OFFSET_Z_MSB__REG,
					&v_data2_u8r, BNO055_ONE_U8X);
					if (com_rslt == SUCCESS) {
						v_data1_u8r = ((s8)
						(accel_offset->z >>
						BNO055_SHIFT_8_POSITION)
						& BNO055_SIC_HEX_0_0_F_F_DATA);
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_ACCEL_OFFSET_Z_MSB,
						v_data1_u8r);
						com_rslt +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_ACCEL_OFFSET_Z_MSB__REG,
						&v_data2_u8r, BNO055_ONE_U8X);
					}

				/*write accel radius value*/
					com_rslt +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_ACCEL_RADIUS_LSB__REG,
					&v_data2_u8r, BNO055_ONE_U8X);
					if (com_rslt == SUCCESS) {
						v_data1_u8r = ((s8)
						(accel_offset->r
						& BNO055_SIC_HEX_0_0_F_F_DATA));
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_ACCEL_RADIUS_LSB,
						v_data1_u8r);
						com_rslt =
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_ACCEL_RADIUS_LSB__REG,
						&v_data2_u8r, BNO055_ONE_U8X);
					}

					com_rslt +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_ACCEL_RADIUS_MSB__REG,
					&v_data2_u8r, BNO055_ONE_U8X);
					if (com_rslt == SUCCESS) {
						v_data1_u8r = ((s8)
						(accel_offset->r >>
						BNO055_SHIFT_8_POSITION)
						& BNO055_SIC_HEX_0_0_F_F_DATA);
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_ACCEL_RADIUS_MSB,
						v_data1_u8r);
						com_rslt =
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_ACCEL_RADIUS_MSB__REG,
						&v_data2_u8r, BNO055_ONE_U8X);
					}
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	}
	if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
		/* set the operation mode
		of previous operation mode*/
		com_rslt += bno055_set_operation_mode
		(v_prev_opmode_u8);
	return com_rslt;
}

/*!
 *	@brief This API is used to read mag offset
 *	offset form register 0x69 to 0x6A
 *
 *	@param mag_offset :  The value of mag offset and radius
 *
 *	    bno055_mag_offset_t   |     result
 *      ------------------- | ----------------
 *	             x          |  mag offset x
 *               y          |  mag offset y
 *	             z          |  mag offset z
 *               r          |  mag radius r
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note  The range of the magnetometer offset is +/-6400 in LSB
 */

BNO055_RETURN_FUNCTION_TYPE bno055_read_mag_offset(
struct bno055_mag_offset_t  *mag_offset)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	/* Array holding the mag offset values
	v_data_u8[INDEX_ZERO] - offset x->LSB
	v_data_u8[INDEX_ONE] - offset x->MSB
	v_data_u8[INDEX_TWO] - offset y->LSB
	v_data_u8[INDEX_THREE] - offset y->MSB
	v_data_u8[INDEX_FOUR] - offset z->LSB
	v_data_u8[INDEX_FIVE] - offset z->MSB
	*/
	u8 v_data_u8[ARRAY_SIZE_SIX] = {
	BNO055_ZERO_U8X, BNO055_ZERO_U8X,
	BNO055_ZERO_U8X, BNO055_ZERO_U8X,
	BNO055_ZERO_U8X, BNO055_ZERO_U8X};
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, mag offset is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read mag offset value it the six bytes of data */
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_MAG_OFFSET_X_LSB__REG,
			v_data_u8, BNO055_SIX_U8X);
			if (com_rslt == SUCCESS) {
				/* Read mag x offset value*/
				v_data_u8[INDEX_ZERO] =
				BNO055_GET_BITSLICE(v_data_u8[INDEX_ZERO],
				BNO055_MAG_OFFSET_X_LSB);
				v_data_u8[INDEX_ONE] =
				BNO055_GET_BITSLICE(v_data_u8[INDEX_ONE],
				BNO055_MAG_OFFSET_X_MSB);
				mag_offset->x = (s16)((((s32)
				(s8)(v_data_u8[INDEX_ONE])) <<
				(BNO055_SHIFT_8_POSITION)) |
				(v_data_u8[INDEX_ZERO]));

				/* Read mag y offset value*/
				v_data_u8[INDEX_TWO] =
				BNO055_GET_BITSLICE(v_data_u8[INDEX_TWO],
				BNO055_MAG_OFFSET_Y_LSB);
				v_data_u8[INDEX_THREE] =
				BNO055_GET_BITSLICE(v_data_u8[INDEX_THREE],
				BNO055_MAG_OFFSET_Y_MSB);
				mag_offset->y = (s16)((((s32)
				(s8)(v_data_u8[INDEX_THREE])) <<
				(BNO055_SHIFT_8_POSITION))
				| (v_data_u8[INDEX_TWO]));

				/* Read mag z offset value*/
				v_data_u8[INDEX_FOUR] =
				BNO055_GET_BITSLICE(v_data_u8[INDEX_FOUR],
				BNO055_MAG_OFFSET_Z_LSB);
				v_data_u8[INDEX_FIVE] =
				BNO055_GET_BITSLICE(v_data_u8[INDEX_FIVE],
				BNO055_MAG_OFFSET_Z_MSB);
				mag_offset->z = (s16)((((s32)
				(s8)(v_data_u8[INDEX_FIVE])) <<
				(BNO055_SHIFT_8_POSITION))
				| (v_data_u8[INDEX_FOUR]));

				/* Read mag radius value
				it the two bytes of data */
				com_rslt += p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_MAG_RADIUS_LSB__REG,
				v_data_u8, BNO055_TWO_U8X);
				if (com_rslt == SUCCESS) {
					/* Array holding the mag radius values
					v_data_u8[INDEX_ZERO] - radius->LSB
					v_data_u8[INDEX_ONE] - radius->MSB
					*/
					v_data_u8[INDEX_ZERO] =
					BNO055_GET_BITSLICE(
						v_data_u8[INDEX_ZERO],
					BNO055_MAG_RADIUS_LSB);
					v_data_u8[INDEX_ONE] =
					BNO055_GET_BITSLICE(
						v_data_u8[INDEX_ONE],
					BNO055_MAG_RADIUS_MSB);
					mag_offset->r = (s16)((((s32)
					(s8)(v_data_u8[INDEX_ONE])) <<
					(BNO055_SHIFT_8_POSITION)) |
					(v_data_u8[INDEX_ZERO]));
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}

/*!
 *	@brief This API is used to read mag offset
 *	offset form register 0x69 to 0x6A
 *
 *	@param mag_offset :  The value of mag offset and radius
 *
 *	    bno055_mag_offset_t |     result
 *      ------------------- | ----------------
 *	             x          |  mag offset x
 *               y          |  mag offset y
 *	             z          |  mag offset z
 *               r          |  mag radius r
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note  The range of the magnetometer offset is +/-6400 in LSB
 */
BNO055_RETURN_FUNCTION_TYPE bno055_write_mag_offset(
struct bno055_mag_offset_t *mag_offset)
{
BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 v_data1_u8r = BNO055_ZERO_U8X;
u8 v_data2_u8r = BNO055_ZERO_U8X;
u8 v_prev_opmode_u8 = OPERATION_MODE_CONFIG;
s8 v_stat_s8 = ERROR;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
	} else {
	/* The write operation effective only if the operation
		mode is in config mode, this part of code is checking the
		current operation mode and set the config mode */
		v_stat_s8 = bno055_get_operation_mode(&v_prev_opmode_u8);
		if (v_stat_s8 == SUCCESS) {
			if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
				v_stat_s8 += bno055_set_operation_mode
				(OPERATION_MODE_CONFIG);
				if (v_stat_s8 == SUCCESS) {
					/* write Mag offset x value*/
					com_rslt =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_MAG_OFFSET_X_LSB__REG,
					&v_data2_u8r, BNO055_ONE_U8X);
					if (com_rslt == SUCCESS) {
						v_data1_u8r = ((s8)
						(mag_offset->x
						& BNO055_SIC_HEX_0_0_F_F_DATA));
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_MAG_OFFSET_X_LSB,
						v_data1_u8r);
						com_rslt +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_MAG_OFFSET_X_LSB__REG,
						&v_data2_u8r, BNO055_ONE_U8X);
					}

					com_rslt +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_MAG_OFFSET_X_MSB__REG,
					&v_data2_u8r, BNO055_ONE_U8X);
					if (com_rslt == SUCCESS) {
						v_data1_u8r = ((s8)
						(mag_offset->x  >>
						BNO055_SHIFT_8_POSITION)
						& BNO055_SIC_HEX_0_0_F_F_DATA);
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_MAG_OFFSET_X_MSB,
						v_data1_u8r);
						com_rslt +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_MAG_OFFSET_X_MSB__REG,
						&v_data2_u8r, BNO055_ONE_U8X);
					}

					/* write Mag offset y value*/
					com_rslt +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_MAG_OFFSET_Y_LSB__REG,
					&v_data2_u8r, BNO055_ONE_U8X);
					if (com_rslt == SUCCESS) {
						v_data1_u8r = ((s8)
						(mag_offset->y &
						BNO055_SIC_HEX_0_0_F_F_DATA));
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_MAG_OFFSET_Y_LSB,
						v_data1_u8r);
						com_rslt +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_MAG_OFFSET_Y_LSB__REG,
						&v_data2_u8r, BNO055_ONE_U8X);
					}

					com_rslt +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_MAG_OFFSET_Y_MSB__REG,
					&v_data2_u8r, BNO055_ONE_U8X);
					if (com_rslt == SUCCESS) {
						v_data1_u8r = ((s8)
						(mag_offset->y >>
						BNO055_SHIFT_8_POSITION)
						& BNO055_SIC_HEX_0_0_F_F_DATA);
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_MAG_OFFSET_Y_MSB,
						v_data1_u8r);
						com_rslt +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_MAG_OFFSET_Y_MSB__REG,
						&v_data2_u8r, BNO055_ONE_U8X);
					}
				/* write Mag offset z value*/
					com_rslt +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_MAG_OFFSET_Z_LSB__REG,
					&v_data2_u8r, BNO055_ONE_U8X);
					if (com_rslt == SUCCESS) {
						v_data1_u8r = ((s8)
						(mag_offset->z &
						BNO055_SIC_HEX_0_0_F_F_DATA));
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_MAG_OFFSET_Z_LSB,
						v_data1_u8r);
						com_rslt +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_MAG_OFFSET_Z_LSB__REG,
						&v_data2_u8r, BNO055_ONE_U8X);
					}

					com_rslt +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_MAG_OFFSET_Z_MSB__REG,
					&v_data2_u8r, BNO055_ONE_U8X);
					if (com_rslt == SUCCESS) {
						v_data1_u8r = ((s8)
						(mag_offset->z >>
						BNO055_SHIFT_8_POSITION)
						& BNO055_SIC_HEX_0_0_F_F_DATA);
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_MAG_OFFSET_Z_MSB,
						v_data1_u8r);
						com_rslt +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_MAG_OFFSET_Z_MSB__REG,
						&v_data2_u8r, BNO055_ONE_U8X);
					}

					/* write Mag radius value*/
					com_rslt +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_MAG_RADIUS_LSB__REG,
					&v_data2_u8r, BNO055_ONE_U8X);
					if (com_rslt == SUCCESS) {
						v_data1_u8r = ((s8)
						(mag_offset->r &
						BNO055_SIC_HEX_0_0_F_F_DATA));
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_MAG_RADIUS_LSB,
						v_data1_u8r);
						com_rslt +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_MAG_RADIUS_LSB__REG,
						&v_data2_u8r, BNO055_ONE_U8X);
					}

					com_rslt +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_MAG_RADIUS_MSB__REG,
					&v_data2_u8r, BNO055_ONE_U8X);
					if (com_rslt == SUCCESS) {
						v_data1_u8r = ((s8)
						(mag_offset->r >>
						BNO055_SHIFT_8_POSITION)
						& BNO055_SIC_HEX_0_0_F_F_DATA);
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_MAG_RADIUS_MSB,
						v_data1_u8r);
						com_rslt +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_MAG_RADIUS_MSB__REG,
						&v_data2_u8r, BNO055_ONE_U8X);
					}
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	}
	if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
		/* set the operation mode
		of previous operation mode*/
		com_rslt += bno055_set_operation_mode
		(v_prev_opmode_u8);
	return com_rslt;
}
/*!
 *	@brief This API is used to read gyro offset
 *	offset form register 0x61 to 0x66
 *
 *	@param gyro_offset : The value of gyro offset
 *
 *	    bno055_gyro_offset_t  |     result
 *      ------------------- | ----------------
 *	             x          |  gyro offset x
 *               y          |  gyro offset y
 *	             z          |  gyro offset z
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note  The range of the gyro offset varies based on
 *	the range of gyro sensor
 *
 *	gyro G range         | offset range
 * --------------------  | ------------
 *  GYRO_RANGE_2000DPS   | +/-32000
 *  GYRO_RANGE_1000DPS   | +/-16000
 *  GYRO_RANGE_500DPS    | +/-8000
 *  GYRO_RANGE_250DPS    | +/-4000
 *  GYRO_RANGE_125DPS    | +/-2000
 *
 *	Gyro range can be configured by using the
 *	bno055_set_gyro_range() function
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_gyro_offset(
struct bno055_gyro_offset_t  *gyro_offset)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	/* Array holding the gyro offset values
	v_data_u8[INDEX_ZERO] - offset x->LSB
	v_data_u8[INDEX_ONE] - offset x->MSB
	v_data_u8[INDEX_TWO] - offset y->LSB
	v_data_u8[INDEX_THREE] - offset y->MSB
	v_data_u8[INDEX_FOUR] - offset z->LSB
	v_data_u8[INDEX_FIVE] - offset z->MSB
	*/
	u8 v_data_u8[ARRAY_SIZE_SIX] = {
	BNO055_ZERO_U8X, BNO055_ZERO_U8X,
	BNO055_ZERO_U8X, BNO055_ZERO_U8X,
	BNO055_ZERO_U8X, BNO055_ZERO_U8X};
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, gyro offset is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			v_stat_s8 = bno055_write_page_id(PAGE_ZERO);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ZERO)) {
			/* Read gyro offset value it the six bytes of data*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_OFFSET_X_LSB__REG,
			v_data_u8, BNO055_SIX_U8X);
			if (com_rslt == SUCCESS) {
				/* Read gyro x offset value*/
				v_data_u8[INDEX_ZERO] =
				BNO055_GET_BITSLICE(v_data_u8[INDEX_ZERO],
				BNO055_GYRO_OFFSET_X_LSB);
				v_data_u8[INDEX_ONE] =
				BNO055_GET_BITSLICE(v_data_u8[INDEX_ONE],
				BNO055_GYRO_OFFSET_X_MSB);
				gyro_offset->x = (s16)((((s32)
				(s8)(v_data_u8[INDEX_ONE])) <<
				(BNO055_SHIFT_8_POSITION)) |
				(v_data_u8[INDEX_ZERO]));

				/* Read gyro y offset value*/
				v_data_u8[INDEX_TWO] =
				BNO055_GET_BITSLICE(v_data_u8[INDEX_TWO],
				BNO055_GYRO_OFFSET_Y_LSB);
				v_data_u8[INDEX_THREE] =
				BNO055_GET_BITSLICE(v_data_u8[INDEX_THREE],
				BNO055_GYRO_OFFSET_Y_MSB);
				gyro_offset->y = (s16)((((s32)
				(s8)(v_data_u8[INDEX_THREE])) <<
				(BNO055_SHIFT_8_POSITION)) |
				(v_data_u8[INDEX_TWO]));

				/* Read gyro z offset value*/
				v_data_u8[INDEX_FOUR] =
				BNO055_GET_BITSLICE(v_data_u8[INDEX_FOUR],
				BNO055_GYRO_OFFSET_Z_LSB);
				v_data_u8[INDEX_FIVE] =
				BNO055_GET_BITSLICE(v_data_u8[INDEX_FIVE],
				BNO055_GYRO_OFFSET_Z_MSB);
				gyro_offset->z = (s16)((((s32)
				(s8)(v_data_u8[INDEX_FIVE])) <<
				(BNO055_SHIFT_8_POSITION)) |
				(v_data_u8[INDEX_FOUR]));
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to read gyro offset
 *	offset form register 0x61 to 0x66
 *
 *	@param gyro_offset : The value of gyro offset
 *
 *	    bno055_gyro_offset_t  |     result
 *      ------------------- | ----------------
 *	             x          |  gyro offset x
 *               y          |  gyro offset y
 *	             z          |  gyro offset z
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note  The range of the gyro offset varies based on
 *	the range of gyro sensor
 *
 *	gyro G range         | offset range
 * --------------------  | ------------
 *  GYRO_RANGE_2000DPS   | +/-32000
 *  GYRO_RANGE_1000DPS   | +/-16000
 *  GYRO_RANGE_500DPS    | +/-8000
 *  GYRO_RANGE_250DPS    | +/-4000
 *  GYRO_RANGE_125DPS    | +/-2000
 *
 *	Gyro range can be configured by using the
 *	bno055_set_gyro_range() function
 */
BNO055_RETURN_FUNCTION_TYPE bno055_write_gyro_offset(
struct bno055_gyro_offset_t  *gyro_offset)
{
BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 v_data1_u8r = BNO055_ZERO_U8X;
u8 v_data2_u8r = BNO055_ZERO_U8X;
u8 v_prev_opmode_u8 = OPERATION_MODE_CONFIG;
s8 v_stat_s8 = ERROR;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
	} else {
	/* The write operation effective only if the operation
		mode is in config mode, this part of code is checking the
		current operation mode and set the config mode */
	v_stat_s8 = bno055_get_operation_mode(&v_prev_opmode_u8);
	if (v_stat_s8 == SUCCESS) {
		if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
				v_stat_s8 += bno055_set_operation_mode
				(OPERATION_MODE_CONFIG);
				if (v_stat_s8 == SUCCESS) {
					/* write gryo offset x value*/
					com_rslt =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_GYRO_OFFSET_X_LSB__REG,
					&v_data2_u8r, BNO055_ONE_U8X);
					if (com_rslt == SUCCESS) {
						v_data1_u8r = ((s8)
						(gyro_offset->x
						& BNO055_SIC_HEX_0_0_F_F_DATA));
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_GYRO_OFFSET_X_LSB,
						v_data1_u8r);
						com_rslt +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_GYRO_OFFSET_X_LSB__REG,
						&v_data2_u8r, BNO055_ONE_U8X);
					}

					com_rslt +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_GYRO_OFFSET_X_MSB__REG,
					&v_data2_u8r, BNO055_ONE_U8X);
					if (com_rslt == SUCCESS) {
						v_data1_u8r = ((s8)
						(gyro_offset->x  >>
						BNO055_SHIFT_8_POSITION)
						& BNO055_SIC_HEX_0_0_F_F_DATA);
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_GYRO_OFFSET_X_MSB,
						v_data1_u8r);
						com_rslt +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_GYRO_OFFSET_X_MSB__REG,
						&v_data2_u8r, BNO055_ONE_U8X);
					}

					/* write gryo offset y value*/
					com_rslt +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_GYRO_OFFSET_Y_LSB__REG,
					&v_data2_u8r, BNO055_ONE_U8X);
					if (com_rslt == SUCCESS) {
						v_data1_u8r = ((s8)
						(gyro_offset->y
						& BNO055_SIC_HEX_0_0_F_F_DATA));
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_GYRO_OFFSET_Y_LSB,
						v_data1_u8r);
						com_rslt +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_GYRO_OFFSET_Y_LSB__REG,
						&v_data2_u8r, BNO055_ONE_U8X);
					}

					com_rslt +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_GYRO_OFFSET_Y_MSB__REG,
					&v_data2_u8r, BNO055_ONE_U8X);
					if (com_rslt == SUCCESS) {
						v_data1_u8r = ((s8)
						(gyro_offset->y >>
						BNO055_SHIFT_8_POSITION)
						& BNO055_SIC_HEX_0_0_F_F_DATA);
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_GYRO_OFFSET_Y_MSB,
						v_data1_u8r);
						com_rslt +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_GYRO_OFFSET_Y_MSB__REG,
						&v_data2_u8r, BNO055_ONE_U8X);
					}
				/* write gryo offset z value*/
					com_rslt +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_GYRO_OFFSET_Z_LSB__REG,
					&v_data2_u8r, BNO055_ONE_U8X);
					if (com_rslt == SUCCESS) {
						v_data1_u8r = ((s8)
						(gyro_offset->z
						& BNO055_SIC_HEX_0_0_F_F_DATA));
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_GYRO_OFFSET_Z_LSB,
						v_data1_u8r);
						com_rslt +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_GYRO_OFFSET_Z_LSB__REG,
						&v_data2_u8r, BNO055_ONE_U8X);
					}

					com_rslt +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_GYRO_OFFSET_Z_MSB__REG,
					&v_data2_u8r, BNO055_ONE_U8X);
					if (com_rslt == SUCCESS) {
						v_data1_u8r = ((s8)
						(gyro_offset->z >>
						BNO055_SHIFT_8_POSITION)
						& BNO055_SIC_HEX_0_0_F_F_DATA);
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_GYRO_OFFSET_Z_MSB,
						v_data1_u8r);
						com_rslt +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_GYRO_OFFSET_Z_MSB__REG,
						&v_data2_u8r, BNO055_ONE_U8X);
					}
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	}
	if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
		/* set the operation mode
		of previous operation mode*/
		com_rslt += bno055_set_operation_mode
		(v_prev_opmode_u8);
	return com_rslt;
}
/********************************************************/
 /************** PAGE1 Functions *********************/
/********************************************************/
/*!
 *	@brief This API used to read the accel range
 *	from page one register from 0x08 bit 0 and 1
 *
 *	@param v_accel_range_u8 : The value of accel range
 *		  v_accel_range_u8 |   result
 *       ----------------- | --------------
 *              0x00       | ACCEL_RANGE_2G
 *              0x01       | ACCEL_RANGE_4G
 *              0x02       | ACCEL_RANGE_8G
 *              0x03       | ACCEL_RANGE_16G
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_range(
u8 *v_accel_range_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel range is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			v_stat_s8 = bno055_write_page_id(PAGE_ONE);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ONE)) {
			/* Read the value of accel g range */
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_RANGE__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_accel_range_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_ACCEL_RANGE);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to write the accel range
 *	from page one register from 0x08 bit 0 and 1
 *
 *	@param v_accel_range_u8 : The value of accel range
 *
 *		  v_accel_range_u8 |   result
 *       ----------------- | --------------
 *              0x00       | ACCEL_RANGE_2G
 *              0x01       | ACCEL_RANGE_4G
 *              0x02       | ACCEL_RANGE_8G
 *              0x03       | ACCEL_RANGE_16G
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_range(
u8 v_accel_range_u8)
{
BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 v_data_u8r = BNO055_ZERO_U8X;
s8 v_stat_s8 = ERROR;
u8 v_prev_opmode_u8 = OPERATION_MODE_CONFIG;
s8 v_pg_stat_s8 = ERROR;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
} else {
/* The write operation effective only if the operation
mode is in config mode, this part of code is checking the
current operation mode and set the config mode */
v_stat_s8 = bno055_get_operation_mode(&v_prev_opmode_u8);
if (v_stat_s8 == SUCCESS) {
	if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
		v_stat_s8 += bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);
		if (v_stat_s8 == SUCCESS) {
			/* Write page as one */
			v_pg_stat_s8 = bno055_write_page_id(PAGE_ONE);
			if (v_pg_stat_s8 == SUCCESS) {
				if (v_accel_range_u8 < BNO055_FIVE_U8X) {
					/* Write the value of accel range*/
					com_rslt =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_ACCEL_RANGE__REG,
					&v_data_u8r, BNO055_ONE_U8X);
					if (com_rslt == SUCCESS) {
						v_data_u8r = BNO055_SET_BITSLICE
						(v_data_u8r,
						BNO055_ACCEL_RANGE,
						v_accel_range_u8);
						com_rslt +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_ACCEL_RANGE__REG,
						&v_data_u8r, BNO055_ONE_U8X);
					}
				} else {
				com_rslt = E_BNO055_OUT_OF_RANGE;
				}
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	} else {
	com_rslt = ERROR;
	}
}
if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
	/* set the operation mode of
	previous operation mode*/
	com_rslt += bno055_set_operation_mode
	(v_prev_opmode_u8);
return com_rslt;
}
/*!
 *	@brief This API used to read the accel bandwidth
 *	from page one register from 0x08 bit 2 to 4
 *
 *	@param v_accel_bw_u8 : The value of accel bandwidth
 *
 *		     v_accel_bw_u8 |     result
 *       ----------------- | ---------------
 *              0x00       | ACCEL_BW_7_81HZ
 *              0x01       | ACCEL_BW_15_63HZ
 *              0x02       | ACCEL_BW_31_25HZ
 *              0x03       | ACCEL_BW_62_5HZ
 *              0x04       | ACCEL_BW_125HZ
 *              0x05       | ACCEL_BW_250HZ
 *              0x06       | ACCEL_BW_500HZ
 *              0x07       | ACCEL_BW_1000HZ
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_bw(
u8 *v_accel_bw_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel bandwidth is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			v_stat_s8 = bno055_write_page_id(PAGE_ONE);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ONE)) {
			/* Read the value of accel bandwidth */
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_BW__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_accel_bw_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_ACCEL_BW);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to write the accel bandwidth
 *	from page one register from 0x08 bit 2 to 4
 *
 *	@param v_accel_bw_u8 : The value of accel bandwidth
 *
 *		     v_accel_bw_u8 |     result
 *       ----------------- | ---------------
 *              0x00       | ACCEL_BW_7_81HZ
 *              0x01       | ACCEL_BW_15_63HZ
 *              0x02       | ACCEL_BW_31_25HZ
 *              0x03       | ACCEL_BW_62_5HZ
 *              0x04       | ACCEL_BW_125HZ
 *              0x05       | ACCEL_BW_250HZ
 *              0x06       | ACCEL_BW_500HZ
 *              0x07       | ACCEL_BW_1000HZ
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_bw(
u8 v_accel_bw_u8)
{
BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 v_data_u8r = BNO055_ZERO_U8X;
s8 v_stat_s8 = ERROR;
u8 v_prev_opmode_u8 = OPERATION_MODE_CONFIG;
s8 v_pg_stat_s8 = ERROR;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
} else {
/* The write operation effective only if the operation
mode is in config mode, this part of code is checking the
current operation mode and set the config mode */
v_stat_s8 = bno055_get_operation_mode(&v_prev_opmode_u8);
if (v_stat_s8 == SUCCESS) {
	if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
		v_stat_s8 += bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);
		if (v_stat_s8 == SUCCESS) {
			/* Write page as one */
			v_pg_stat_s8 = bno055_write_page_id(PAGE_ONE);
			if (v_pg_stat_s8 == SUCCESS) {
				if (v_accel_bw_u8 < BNO055_EIGHT_U8X) {
					/* Write the accel */
					com_rslt =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_ACCEL_BW__REG,
					&v_data_u8r, BNO055_ONE_U8X);
					if (com_rslt == SUCCESS) {
						v_data_u8r = BNO055_SET_BITSLICE
						(v_data_u8r, BNO055_ACCEL_BW,
						v_accel_bw_u8);
						com_rslt +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_ACCEL_BW__REG,
						&v_data_u8r, BNO055_ONE_U8X);
					}
				} else {
				com_rslt = E_BNO055_OUT_OF_RANGE;
				}
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	} else {
	com_rslt = ERROR;
	}
}
if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
	/* set the operation mode of
	previous operation mode*/
	com_rslt += bno055_set_operation_mode
	(v_prev_opmode_u8);
return com_rslt;
}
/*!
 *	@brief This API used to read the accel power mode
 *	from page one register from 0x08 bit 5 to 7
 *
 *	@param v_accel_power_mode_u8 : The value of accel power mode
 * v_accel_power_mode_u8 |   result
 *   -----------------   | -------------
 *              0x00     | ACCEL_NORMAL
 *              0x01     | ACCEL_SUSPEND
 *              0x02     | ACCEL_LOWPOWER_1
 *              0x03     | ACCEL_STANDBY
 *              0x04     | ACCEL_LOWPOWER_2
 *              0x05     | ACCEL_DEEPSUSPEND
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_power_mode(
u8 *v_accel_power_mode_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel power mode is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			v_stat_s8 = bno055_write_page_id(PAGE_ONE);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ONE)) {
			/* Read the value of accel bandwidth */
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_POWER_MODE__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_accel_power_mode_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_ACCEL_POWER_MODE);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to write the accel power mode
 *	from page one register from 0x08 bit 5 to 7
 *
 *	@param v_accel_power_mode_u8 : The value of accel power mode
 * v_accel_power_mode_u8 |   result
 *   -----------------   | -------------
 *              0x00     | ACCEL_NORMAL
 *              0x01     | ACCEL_SUSPEND
 *              0x02     | ACCEL_LOWPOWER_1
 *              0x03     | ACCEL_STANDBY
 *              0x04     | ACCEL_LOWPOWER_2
 *              0x05     | ACCEL_DEEPSUSPEND
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_power_mode(
u8 v_accel_power_mode_u8)
{
BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 v_data_u8r = BNO055_ZERO_U8X;
s8 v_stat_s8 = ERROR;
u8 v_prev_opmode_u8 = OPERATION_MODE_CONFIG;
s8 v_pg_stat_s8 = ERROR;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
} else {
/* The write operation effective only if the operation
mode is in config mode, this part of code is checking the
current operation mode and set the config mode */
v_stat_s8 = bno055_get_operation_mode(&v_prev_opmode_u8);
if (v_stat_s8 == SUCCESS) {
	if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
		v_stat_s8 += bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);
		if (v_stat_s8 == SUCCESS) {
			/* Write page as one */
			v_pg_stat_s8  = bno055_write_page_id(PAGE_ONE);
			if (v_pg_stat_s8 == SUCCESS) {
				if (v_accel_power_mode_u8 < BNO055_SIX_U8X) {
					/* Write the value of accel bandwidth*/
					com_rslt =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_ACCEL_POWER_MODE__REG,
					&v_data_u8r, BNO055_ONE_U8X);
					if (com_rslt == SUCCESS) {
						v_data_u8r = BNO055_SET_BITSLICE
						(v_data_u8r,
						BNO055_ACCEL_POWER_MODE,
						v_accel_power_mode_u8);
						com_rslt +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_ACCEL_POWER_MODE__REG,
						&v_data_u8r, BNO055_ONE_U8X);
					}
				} else {
				com_rslt = E_BNO055_OUT_OF_RANGE;
				}
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	} else {
	com_rslt = ERROR;
	}
}
if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
	/* set the operation mode of
	previous operation mode*/
	com_rslt += bno055_set_operation_mode
	(v_prev_opmode_u8);
return com_rslt;
}
/*!
 *	@brief This API used to read the mag output data rate
 *	from page one register from 0x09 bit 0 to 2
 *
 *	@param v_mag_data_output_rate_u8 : The value of mag output data rate
 *
 *  v_mag_data_output_rate_u8 |   result
 *  ----------------------    |----------------------
 *     0x00                   | MAG_DATA_OUTPUT_RATE_2HZ
 *     0x01                   | MAG_DATA_OUTPUT_RATE_6HZ
 *     0x02                   | MAG_DATA_OUTPUT_RATE_8HZ
 *     0x03                   | MAG_DATA_OUTPUT_RATE_10HZ
 *     0x04                   | MAG_DATA_OUTPUT_RATE_15HZ
 *     0x05                   | MAG_DATA_OUTPUT_RATE_20HZ
 *     0x06                   | MAG_DATA_OUTPUT_RATE_25HZ
 *     0x07                   | MAG_DATA_OUTPUT_RATE_30HZ
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_mag_data_output_rate(
u8 *v_mag_data_output_rate_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, output data rate
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			v_stat_s8 = bno055_write_page_id(PAGE_ONE);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ONE)) {
			/* Read the mag output data rate*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_MAG_DATA_OUTPUT_RATE__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_mag_data_output_rate_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_MAG_DATA_OUTPUT_RATE);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to write the mag output data rate
 *	from page one register from 0x09 bit 0 to 2
 *
 *	@param v_mag_data_output_rate_u8 : The value of mag output data rate
 *
 *  v_mag_data_output_rate_u8 |   result
 *  ----------------------    |----------------------
 *     0x00                   | MAG_DATA_OUTPUT_RATE_2HZ
 *     0x01                   | MAG_DATA_OUTPUT_RATE_6HZ
 *     0x02                   | MAG_DATA_OUTPUT_RATE_8HZ
 *     0x03                   | MAG_DATA_OUTPUT_RATE_10HZ
 *     0x04                   | MAG_DATA_OUTPUT_RATE_15HZ
 *     0x05                   | MAG_DATA_OUTPUT_RATE_20HZ
 *     0x06                   | MAG_DATA_OUTPUT_RATE_25HZ
 *     0x07                   | MAG_DATA_OUTPUT_RATE_30HZ
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_mag_data_output_rate(
u8 v_mag_data_output_rate_u8)
{
BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 v_data_u8r = BNO055_ZERO_U8X;
s8 v_stat_s8 = ERROR;
s8 v_pg_stat_s8 = ERROR;
u8 v_prev_opmode_u8 = OPERATION_MODE_CONFIG;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
} else {
/* The write operation effective only if the operation
mode is in config mode, this part of code is checking the
current operation mode and set the config mode */
v_stat_s8 = bno055_get_operation_mode(&v_prev_opmode_u8);
if (v_stat_s8 == SUCCESS) {
	if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
		v_stat_s8 += bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);
	if (v_stat_s8 == SUCCESS) {
		/* Write page as one */
		v_pg_stat_s8 = bno055_write_page_id(PAGE_ONE);
		if (v_pg_stat_s8 == SUCCESS) {
			if (v_mag_data_output_rate_u8
				< BNO055_EIGHT_U8X) {
				/* Write the value of
				mag output data rate*/
				com_rslt =
				p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_MAG_DATA_OUTPUT_RATE__REG,
				&v_data_u8r, BNO055_ONE_U8X);
				if (com_rslt == SUCCESS) {
					v_data_u8r = BNO055_SET_BITSLICE
					(v_data_u8r,
					BNO055_MAG_DATA_OUTPUT_RATE,
					v_mag_data_output_rate_u8);
					com_rslt +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_MAG_DATA_OUTPUT_RATE__REG,
					&v_data_u8r, BNO055_ONE_U8X);
				}
			} else {
			com_rslt = E_BNO055_OUT_OF_RANGE;
			}
		} else {
		com_rslt = ERROR;
		}
	} else {
	com_rslt = ERROR;
	}
} else {
com_rslt = ERROR;
}
}
if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
	/* set the operation mode of
	previous operation mode*/
	com_rslt += bno055_set_operation_mode
	(v_prev_opmode_u8);
return com_rslt;
}
/*!
 *	@brief This API used to read the mag operation mode
 *	from page one register from 0x09 bit 3 to 4
 *
 *	@param v_mag_operation_mode_u8 : The value of mag operation mode
 *
 *  v_mag_operation_mode_u8  |      result
 * ------------------------- |--------------------------
 *     0x00                  | MAG_OPR_MODE_LOWPOWER
 *     0x01                  | MAG_OPR_MODE_REGULAR
 *     0x02                  | MAG_OPR_MODE_ENHANCED_REGULAR
 *     0x03                  | MAG_OPR_MODE_HIGH_ACCURACY
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_mag_operation_mode(
u8 *v_mag_operation_mode_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, mag operation mode is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			v_stat_s8 = bno055_write_page_id(PAGE_ONE);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ONE)) {
			/* Read the value of mag operation mode*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_MAG_OPERATION_MODE__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_mag_operation_mode_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_MAG_OPERATION_MODE);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to write the mag operation mode
 *	from page one register from 0x09 bit 3 to 4
 *
 *	@param v_mag_operation_mode_u8 : The value of mag operation mode
 *
 *  v_mag_operation_mode_u8  |      result
 * ------------------------- |--------------------------
 *     0x00                  | MAG_OPR_MODE_LOWPOWER
 *     0x01                  | MAG_OPR_MODE_REGULAR
 *     0x02                  | MAG_OPR_MODE_ENHANCED_REGULAR
 *     0x03                  | MAG_OPR_MODE_HIGH_ACCURACY
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_mag_operation_mode(
u8 v_mag_operation_mode_u8)
{
BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 v_data_u8r = BNO055_ZERO_U8X;
s8 v_stat_s8 = ERROR;
s8 v_pg_stat_s8 = ERROR;
u8 v_prev_opmode_u8 = OPERATION_MODE_CONFIG;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
} else {
/* The write operation effective only if the operation
mode is in config mode, this part of code is checking the
current operation mode and set the config mode */
v_stat_s8 = bno055_get_operation_mode(&v_prev_opmode_u8);
if (v_stat_s8 == SUCCESS) {
	if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
		v_stat_s8 += bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);
		if (v_stat_s8 == SUCCESS) {
			/* Write page as one */
			v_pg_stat_s8 = bno055_write_page_id(PAGE_ONE);
			if (v_pg_stat_s8 == SUCCESS) {
				if (v_mag_operation_mode_u8
					< BNO055_FIVE_U8X) {
					/* Write the value
					of mag operation mode*/
					com_rslt =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_MAG_OPERATION_MODE__REG,
					&v_data_u8r, BNO055_ONE_U8X);
					if (com_rslt == SUCCESS) {
						v_data_u8r = BNO055_SET_BITSLICE
						(v_data_u8r,
						BNO055_MAG_OPERATION_MODE,
						v_mag_operation_mode_u8);
						com_rslt +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_MAG_OPERATION_MODE__REG,
						&v_data_u8r, BNO055_ONE_U8X);
					}
				} else {
				com_rslt = E_BNO055_OUT_OF_RANGE;
				}
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	} else {
	com_rslt = ERROR;
	}
}
if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
	/* set the operation mode of
	previous operation mode*/
	com_rslt += bno055_set_operation_mode
	(v_prev_opmode_u8);
return com_rslt;
}
/*!
 *	@brief This API used to read the mag power mode
 *	from page one register from 0x09 bit 4 to 6
 *
 *	@param v_mag_power_mode_u8 : The value of mag power mode
 *
 * v_mag_power_mode_u8 |   result
 * --------------------|-----------------
 *     0x00            | MAG_POWER_MODE_NORMAL
 *     0x01            | MAG_POWER_MODE_SLEEP
 *     0x02            | MAG_POWER_MODE_SUSPEND
 *     0x03            | MAG_POWER_MODE_FORCE_MODE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_mag_power_mode(
u8 *v_mag_power_mode_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, mag power mode is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			v_stat_s8 = bno055_write_page_id(PAGE_ONE);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ONE)) {
			/* Read the value of mag power mode */
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_MAG_POWER_MODE__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_mag_power_mode_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_MAG_POWER_MODE);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to write the mag power mode
 *	from page one register from 0x09 bit 4 to 6
 *
 *	@param v_mag_power_mode_u8 : The value of mag power mode
 *
 * v_mag_power_mode_u8 |   result
 * --------------------|-----------------
 *     0x00            | MAG_POWER_MODE_NORMAL
 *     0x01            | MAG_POWER_MODE_SLEEP
 *     0x02            | MAG_POWER_MODE_SUSPEND
 *     0x03            | MAG_POWER_MODE_FORCE_MODE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_mag_power_mode(
u8 v_mag_power_mode_u8)
{
BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 v_data_u8r = BNO055_ZERO_U8X;
s8 v_stat_s8 = ERROR;
s8 v_pg_stat_s8 = ERROR;
u8 v_prev_opmode_u8 = OPERATION_MODE_CONFIG;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
} else {
/* The write operation effective only if the operation
mode is in config mode, this part of code is checking the
current operation mode and set the config mode */
v_stat_s8 = bno055_get_operation_mode(&v_prev_opmode_u8);
if (v_stat_s8 == SUCCESS) {
	if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
		v_stat_s8 += bno055_set_operation_mode(
		OPERATION_MODE_CONFIG);
	if (v_stat_s8 == SUCCESS) {
		/* Write page as one */
		v_pg_stat_s8 = bno055_write_page_id(PAGE_ONE);
		if (v_pg_stat_s8 == SUCCESS) {
			if (v_mag_power_mode_u8 < BNO055_FOUR_U8X) {
				/* Write the value of mag power mode*/
				com_rslt =
				p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_MAG_POWER_MODE__REG,
				&v_data_u8r, BNO055_ONE_U8X);
				if (com_rslt == SUCCESS) {
					v_data_u8r = BNO055_SET_BITSLICE
					(v_data_u8r,
					BNO055_MAG_POWER_MODE,
					v_mag_power_mode_u8);
					com_rslt +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_MAG_POWER_MODE__REG,
					&v_data_u8r, BNO055_ONE_U8X);
				}
			} else {
			com_rslt = E_BNO055_OUT_OF_RANGE;
			}
		} else {
		com_rslt = ERROR;
		}
	} else {
	com_rslt = ERROR;
	}
} else {
	com_rslt = ERROR;
}
}
if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
	/* set the operation mode of
	previous operation mode*/
	com_rslt += bno055_set_operation_mode
	(v_prev_opmode_u8);
return com_rslt;
}
/*!
 *	@brief This API used to read the gyro range
 *	from page one register from 0x0A bit 0 to 3
 *
 *	@param v_gyro_range_u8 : The value of gyro range
 *
 *     v_gyro_range_u8 |   result
 * --------------------|-----------------
 *     0x00            | GYRO_RANGE_2000DPS
 *     0x01            | GYRO_RANGE_1000DPS
 *     0x02            | GYRO_RANGE_500DPS
 *     0x03            | GYRO_RANGE_250DPS
 *     0x04            | GYRO_RANGE_125DPS
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_range(
u8 *v_gyro_range_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, gyro range is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			v_stat_s8 = bno055_write_page_id(PAGE_ONE);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ONE)) {
			/* Read the value of gyro range */
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_RANGE__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_gyro_range_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_GYRO_RANGE);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to write the gyro range
 *	from page one register from 0x0A bit 0 to 3
 *
 *	@param v_gyro_range_u8 : The value of gyro range
 *
 *     v_gyro_range_u8 |   result
 * --------------------|-----------------
 *     0x00            | GYRO_RANGE_2000DPS
 *     0x01            | GYRO_RANGE_1000DPS
 *     0x02            | GYRO_RANGE_500DPS
 *     0x03            | GYRO_RANGE_250DPS
 *     0x04            | GYRO_RANGE_125DPS
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_range(
u8 v_gyro_range_u8)
{
BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 v_data_u8r = BNO055_ZERO_U8X;
s8 v_stat_s8 = ERROR;
s8 v_pg_stat_s8 = ERROR;
u8 v_prev_opmode_u8 = OPERATION_MODE_CONFIG;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
} else {
/* The write operation effective only if the operation
mode is in config mode, this part of code is checking the
current operation mode and set the config mode */
v_stat_s8 = bno055_get_operation_mode(&v_prev_opmode_u8);
if (v_stat_s8 == SUCCESS) {
	if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
		v_stat_s8 += bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);
		if (v_stat_s8 == SUCCESS) {
			/* Write page as one */
			v_pg_stat_s8 = bno055_write_page_id(PAGE_ONE);
			if (v_pg_stat_s8 == SUCCESS) {
				if (v_gyro_range_u8 < BNO055_FIVE_U8X) {
					/* Write the value of gyro range*/
					com_rslt =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_GYRO_RANGE__REG,
					&v_data_u8r, BNO055_ONE_U8X);
					if (com_rslt == SUCCESS) {
						v_data_u8r = BNO055_SET_BITSLICE
						(v_data_u8r,
						BNO055_GYRO_RANGE,
						v_gyro_range_u8);
						com_rslt +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_GYRO_RANGE__REG,
						&v_data_u8r, BNO055_ONE_U8X);
					}
				} else {
				com_rslt = E_BNO055_OUT_OF_RANGE;
				}
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	} else {
	com_rslt = ERROR;
	}
}
if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
	/* set the operation mode of
	previous operation mode*/
	com_rslt += bno055_set_operation_mode
	(v_prev_opmode_u8);
return com_rslt;
}
/*!
 *	@brief This API used to read the gyro bandwidth
 *	from page one register from 0x0A bit 3 to 5
 *
 *	@param v_gyro_bw_u8 : The value of gyro bandwidth
 *
 *     v_gyro_bw_u8    |   result
 * --------------------|-----------------
 *     0x00            | GYRO_BW_523HZ
 *     0x01            | GYRO_BW_230HZ
 *     0x02            | GYRO_BW_116HZ
 *     0x03            | GYRO_BW_47HZ
 *     0x04            | GYRO_BW_23HZ
 *     0x05            | GYRO_BW_12HZ
 *     0x06            | GYRO_BW_64HZ
 *     0x07            | GYRO_BW_32HZ
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_bw(
u8 *v_gyro_bw_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, gyro bandwidth is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			v_stat_s8 = bno055_write_page_id(PAGE_ONE);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ONE)) {
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_BW__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_gyro_bw_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_GYRO_BW);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to write the gyro bandwidth
 *	from page one register from 0x0A bit 3 to 5
 *
 *	@param v_gyro_bw_u8 : The value of gyro bandwidth
 *
 *     v_gyro_bw_u8    |   result
 * --------------------|-----------------
 *     0x00            | GYRO_BW_523HZ
 *     0x01            | GYRO_BW_230HZ
 *     0x02            | GYRO_BW_116HZ
 *     0x03            | GYRO_BW_47HZ
 *     0x04            | GYRO_BW_23HZ
 *     0x05            | GYRO_BW_12HZ
 *     0x06            | GYRO_BW_64HZ
 *     0x07            | GYRO_BW_32HZ
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_bw(
u8 v_gyro_bw_u8)
{
BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 v_data_u8r = BNO055_ZERO_U8X;
s8 v_stat_s8 = ERROR;
u8 gyro_opmode = BNO055_ZERO_U8X;
u8 gyro_auto_sleep_durn = BNO055_ZERO_U8X;
s8 v_pg_stat_s8 = ERROR;
u8 v_prev_opmode_u8 = OPERATION_MODE_CONFIG;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
} else {
/* The write operation effective only if the operation
mode is in config mode, this part of code is checking the
current operation mode and set the config mode */
v_stat_s8 = bno055_get_operation_mode(&v_prev_opmode_u8);
if (v_stat_s8 == SUCCESS) {
	if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
		v_stat_s8 += bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);
	if (v_stat_s8 == SUCCESS) {
		/* Write page as one */
		v_pg_stat_s8 = bno055_write_page_id(PAGE_ONE);
	if (v_pg_stat_s8 == SUCCESS) {
		/* Write the value of gyro bandwidth */
		if ((v_gyro_bw_u8 == BNO055_ZERO_U8X ||
			v_gyro_bw_u8 > BNO055_ZERO_U8X) &&
			v_gyro_bw_u8 < BNO055_EIGHT_U8X) {
			switch (v_gyro_bw_u8) {
			case GYRO_BW_523HZ:
			v_gyro_bw_u8 = GYRO_BW_523HZ;
			break;
			case GYRO_BW_230HZ:
			v_gyro_bw_u8 = GYRO_BW_230HZ;
			break;
			case GYRO_BW_116HZ:
			v_gyro_bw_u8 = GYRO_BW_116HZ;
			break;
			case GYRO_BW_47HZ:
			v_gyro_bw_u8 = GYRO_BW_47HZ;
			break;
			case GYRO_BW_23HZ:
			v_gyro_bw_u8 = GYRO_BW_23HZ;
			break;
			case GYRO_BW_12HZ:
			v_gyro_bw_u8 = GYRO_BW_12HZ;
			break;
			case GYRO_BW_64HZ:
			v_gyro_bw_u8 = GYRO_BW_64HZ;
			break;
			case GYRO_BW_32HZ:
			v_gyro_bw_u8 = GYRO_BW_32HZ;
			break;
			default:
			break;
			}
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_BW__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			if (com_rslt == SUCCESS) {
				v_data_u8r = BNO055_SET_BITSLICE
				(v_data_u8r,
				BNO055_GYRO_BW,
				v_gyro_bw_u8);
				com_rslt +=
				p_bno055->BNO055_BUS_WRITE_FUNC
				(p_bno055->dev_addr,
				BNO055_GYRO_BW__REG,
				&v_data_u8r, BNO055_ONE_U8X);
			}
			com_rslt = bno055_get_gyro_power_mode
			(&gyro_opmode);
			if (com_rslt == SUCCESS) {
				if (gyro_opmode ==
				GYRO_POWER_MODE_ADVANCE_POWERSAVE) {
					com_rslt +=
					bno055_get_gyro_auto_sleep_durn
					(&gyro_auto_sleep_durn);
					if (com_rslt == SUCCESS) {
						com_rslt +=
						bno055_gyro_set_auto_sleep_durn
						(gyro_auto_sleep_durn,
						v_gyro_bw_u8);
					}
				}
			}
		} else {
		com_rslt = E_BNO055_OUT_OF_RANGE;
		}
	} else {
	com_rslt = ERROR;
	}
} else {
com_rslt = ERROR;
}
} else {
com_rslt = ERROR;
}
}
if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
	/* set the operation mode of
	previous operation mode*/
	com_rslt += bno055_set_operation_mode
	(v_prev_opmode_u8);
return com_rslt;
}
/*!
 *	@brief This API used to read the gyro power mode
 *	from page one register from 0x0B bit 0 to 2
 *
 *	@param v_gyro_power_mode_u8 : The value of gyro power mode
 *
 *  v_gyro_power_mode_u8 |          result
 * ----------------------|----------------------------
 *     0x00              | GYRO_OPR_MODE_NORMAL
 *     0x01              | GYRO_OPR_MODE_FASTPOWERUP
 *     0x02              | GYRO_OPR_MODE_DEEPSUSPEND
 *     0x03              | GYRO_OPR_MODE_SUSPEND
 *     0x04              | GYRO_OPR_MODE_ADVANCE_POWERSAVE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_power_mode(
u8 *v_gyro_power_mode_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, gyro power mode is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			v_stat_s8 = bno055_write_page_id(PAGE_ONE);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ONE)) {
			/* Write the value of gyro power mode*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_POWER_MODE__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_gyro_power_mode_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_GYRO_POWER_MODE);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to write the gyro power mode
 *	from page one register from 0x0B bit 0 to 2
 *
 *	@param v_gyro_power_mode_u8 : The value of gyro power mode
 *
 *  v_gyro_power_mode_u8 |          result
 * ----------------------|----------------------------
 *     0x00              | GYRO_OPR_MODE_NORMAL
 *     0x01              | GYRO_OPR_MODE_FASTPOWERUP
 *     0x02              | GYRO_OPR_MODE_DEEPSUSPEND
 *     0x03              | GYRO_OPR_MODE_SUSPEND
 *     0x04              | GYRO_OPR_MODE_ADVANCE_POWERSAVE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_power_mode(
u8 v_gyro_power_mode_u8)
{
BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 v_data_u8r = BNO055_ZERO_U8X;
s8 v_stat_s8 = ERROR;
u8 gyro_auto_sleep_durn = BNO055_ZERO_U8X;
u8 v_gyro_bw_u8 = BNO055_ZERO_U8X;
s8 v_pg_stat_s8 = ERROR;
u8 v_prev_opmode_u8 = OPERATION_MODE_CONFIG;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
} else {
/* The write operation effective only if the operation
mode is in config mode, this part of code is checking the
current operation mode and set the config mode */
v_stat_s8 = bno055_get_operation_mode(&v_prev_opmode_u8);
if (v_stat_s8 == SUCCESS) {
	if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
		v_stat_s8 += bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);
	if (v_stat_s8 == SUCCESS) {
		/* Write page as one */
		v_pg_stat_s8 = bno055_write_page_id(PAGE_ONE);
		if (v_pg_stat_s8 == SUCCESS) {
			/* Write the value of power mode*/
			if ((v_gyro_power_mode_u8 == BNO055_ZERO_U8X ||
			v_gyro_power_mode_u8 > BNO055_ZERO_U8X) &&
			v_gyro_power_mode_u8 < BNO055_FIVE_U8X) {
				switch (v_gyro_power_mode_u8) {
				case GYRO_POWER_MODE_NORMAL:
				v_gyro_power_mode_u8 =
				GYRO_POWER_MODE_NORMAL;
				break;
				case GYRO_POWER_MODE_FASTPOWERUP:
				v_gyro_power_mode_u8 =
				GYRO_POWER_MODE_FASTPOWERUP;
				break;
				case GYRO_POWER_MODE_DEEPSUSPEND:
				v_gyro_power_mode_u8 =
				GYRO_POWER_MODE_DEEPSUSPEND;
				break;
				case GYRO_POWER_MODE_SUSPEND:
				v_gyro_power_mode_u8 =
				GYRO_POWER_MODE_SUSPEND;
				break;
				case GYRO_POWER_MODE_ADVANCE_POWERSAVE:
				com_rslt = bno055_get_gyro_bw
				(&v_gyro_bw_u8);
				com_rslt += bno055_get_gyro_auto_sleep_durn
				(&gyro_auto_sleep_durn);
				if (com_rslt == SUCCESS)
					bno055_gyro_set_auto_sleep_durn
					(gyro_auto_sleep_durn,
					v_gyro_bw_u8);
					com_rslt +=
					bno055_write_page_id(PAGE_ONE);
					v_gyro_power_mode_u8 =
					GYRO_POWER_MODE_ADVANCE_POWERSAVE;
				break;
				default:
				break;
				}
				com_rslt =
				p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_GYRO_POWER_MODE__REG,
				&v_data_u8r, BNO055_ONE_U8X);
				if (com_rslt == SUCCESS) {
					v_data_u8r = BNO055_SET_BITSLICE
					(v_data_u8r,
					BNO055_GYRO_POWER_MODE,
					v_gyro_power_mode_u8);
					com_rslt +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_GYRO_POWER_MODE__REG,
					&v_data_u8r, BNO055_ONE_U8X);
				}
			} else {
			com_rslt = E_BNO055_OUT_OF_RANGE;
			}
		} else {
		com_rslt = ERROR;
		}
	} else {
	com_rslt = ERROR;
	}
} else {
	com_rslt = ERROR;
}
}
if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
	/* set the operation mode of
	previous operation mode*/
	com_rslt += bno055_set_operation_mode
	(v_prev_opmode_u8);
return com_rslt;
}
/*!
 *	@brief This API used to read the accel sleep mode
 *	from page one register from 0x0C bit 0
 *
 *	@param v_sleep_tmr_u8 : The value of accel sleep mode
 *
 *  v_sleep_tmr_u8   |   result
 * ----------------- |------------------------------------
 *     0x00          | enable EventDrivenSampling(EDT)
 *     0x01          | enable Equidistant sampling mode(EST)
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_sleep_tmr_mode(
u8 *v_sleep_tmr_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel sleep mode is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			v_stat_s8 = bno055_write_page_id(PAGE_ONE);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ONE)) {
			/* read the value of accel sleep mode */
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_SLEEP_MODE__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_sleep_tmr_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_ACCEL_SLEEP_MODE);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to write the accel sleep mode
 *	from page one register from 0x0C bit 0
 *
 *	@param v_sleep_tmr_u8 : The value of accel sleep mode
 *
 *  v_sleep_tmr_u8   |   result
 * ----------------- |------------------------------------
 *     0x00          | enable EventDrivenSampling(EDT)
 *     0x01          | enable Equidistant sampling mode(EST)
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_sleep_tmr_mode(
u8 v_sleep_tmr_u8)
{
BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 v_data_u8r = BNO055_ZERO_U8X;
s8 v_stat_s8 = ERROR;
u8 v_prev_opmode_u8 = OPERATION_MODE_CONFIG;
s8 v_pg_stat_s8 = ERROR;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
} else {
	/* The write operation effective only if the operation
	mode is in config mode, this part of code is checking the
	current operation mode and set the config mode */
	v_stat_s8 = bno055_get_operation_mode(&v_prev_opmode_u8);
	if (v_stat_s8 == SUCCESS) {
		if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
			v_stat_s8 += bno055_set_operation_mode
			(OPERATION_MODE_CONFIG);
			if (v_stat_s8 == SUCCESS) {
				/* Write page as one */
				v_pg_stat_s8 = bno055_write_page_id(PAGE_ONE);
				if (v_pg_stat_s8 == SUCCESS) {
					if (v_sleep_tmr_u8 < BNO055_TWO_U8X) {
						/*Write the value
						of accel sleep mode*/
						com_rslt =
						p_bno055->BNO055_BUS_READ_FUNC
						(p_bno055->dev_addr,
						BNO055_ACCEL_SLEEP_MODE__REG,
						&v_data_u8r, BNO055_ONE_U8X);
					if (com_rslt == SUCCESS) {
						v_data_u8r = BNO055_SET_BITSLICE
						(v_data_u8r,
						BNO055_ACCEL_SLEEP_MODE,
						v_sleep_tmr_u8);
						com_rslt +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_ACCEL_SLEEP_MODE__REG,
						&v_data_u8r, BNO055_ONE_U8X);
					}
					} else {
					com_rslt = E_BNO055_OUT_OF_RANGE;
					}
				} else {
				com_rslt = ERROR;
				}
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
}
if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
	/* set the operation mode of
	previous operation mode*/
	com_rslt += bno055_set_operation_mode
	(v_prev_opmode_u8);
return com_rslt;
}
/*!
 *	@brief This API used to read the accel sleep duration
 *	from page one register from 0x0C bit 1 to 4
 *
 *	@param v_sleep_durn_u8 : The value of accel sleep duration
 *
 * v_sleep_durn_u8  |      result
 * ---------------- |-----------------------------
 *     0x05         | BNO055_ACCEL_SLEEP_DURN_0_5MS
 *     0x06         | BNO055_ACCEL_SLEEP_DURN_1MS
 *     0x07         | BNO055_ACCEL_SLEEP_DURN_2MS
 *     0x08         | BNO055_ACCEL_SLEEP_DURN_4MS
 *     0x09         | BNO055_ACCEL_SLEEP_DURN_6MS
 *     0x0A         | BNO055_ACCEL_SLEEP_DURN_10MS
 *     0x0B         | BNO055_ACCEL_SLEEP_DURN_25MS
 *     0x0C         | BNO055_ACCEL_SLEEP_DURN_50MS
 *     0x0D         | BNO055_ACCEL_SLEEP_DURN_100MS
 *     0x0E         | BNO055_ACCEL_SLEEP_DURN_500MS
 *     0x0F         | BNO055_ACCEL_SLEEP_DURN_1S
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_sleep_durn(
u8 *v_sleep_durn_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel sleep duration
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			v_stat_s8 = bno055_write_page_id(PAGE_ONE);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ONE)) {
			/* Read the value of accel sleep duration */
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_SLEEP_DURN__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_sleep_durn_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_ACCEL_SLEEP_DURN);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to write the accel sleep duration
 *	from page one register from 0x0C bit 1 to 4
 *
 *	@param v_sleep_durn_u8 : The value of accel sleep duration
 *
 * v_sleep_durn_u8  |      result
 * ---------------- |-----------------------------
 *     0x05         | BNO055_ACCEL_SLEEP_DURN_0_5MS
 *     0x06         | BNO055_ACCEL_SLEEP_DURN_1MS
 *     0x07         | BNO055_ACCEL_SLEEP_DURN_2MS
 *     0x08         | BNO055_ACCEL_SLEEP_DURN_4MS
 *     0x09         | BNO055_ACCEL_SLEEP_DURN_6MS
 *     0x0A         | BNO055_ACCEL_SLEEP_DURN_10MS
 *     0x0B         | BNO055_ACCEL_SLEEP_DURN_25MS
 *     0x0C         | BNO055_ACCEL_SLEEP_DURN_50MS
 *     0x0D         | BNO055_ACCEL_SLEEP_DURN_100MS
 *     0x0E         | BNO055_ACCEL_SLEEP_DURN_500MS
 *     0x0F         | BNO055_ACCEL_SLEEP_DURN_1S
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_sleep_durn(
u8 v_sleep_durn_u8)
{
BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 v_data_u8r = BNO055_ZERO_U8X;
s8 v_stat_s8 = ERROR;
u8 v_prev_opmode_u8 = OPERATION_MODE_CONFIG;
s8 v_pg_stat_s8 = ERROR;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
} else {
	/* The write operation effective only if the operation
	mode is in config mode, this part of code is checking the
	current operation mode and set the config mode */
	v_stat_s8 = bno055_get_operation_mode(&v_prev_opmode_u8);
	if (v_stat_s8 == SUCCESS) {
		if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
			v_stat_s8 += bno055_set_operation_mode
			(OPERATION_MODE_CONFIG);
			if (v_stat_s8 == SUCCESS) {
				/* Write page as one */
				v_pg_stat_s8 = bno055_write_page_id(PAGE_ONE);
				if (v_pg_stat_s8 == SUCCESS) {
					if (v_sleep_durn_u8 <
					BNO055_SIXTEEN_U8X) {
						/* Write the accel
						sleep duration*/
						com_rslt =
						p_bno055->BNO055_BUS_READ_FUNC
						(p_bno055->dev_addr,
						BNO055_ACCEL_SLEEP_DURN__REG,
						&v_data_u8r, BNO055_ONE_U8X);
					if (com_rslt == SUCCESS) {
						v_data_u8r = BNO055_SET_BITSLICE
						(v_data_u8r,
						BNO055_ACCEL_SLEEP_DURN,
						v_sleep_durn_u8);
						com_rslt +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_ACCEL_SLEEP_DURN__REG,
						&v_data_u8r, BNO055_ONE_U8X);
					}
					} else {
					com_rslt = E_BNO055_OUT_OF_RANGE;
					}
				} else {
				com_rslt = ERROR;
				}
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	}
	if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
		/* set the operation mode of
		previous operation mode*/
		com_rslt += bno055_set_operation_mode
		(v_prev_opmode_u8);
	return com_rslt;
}
/*!
 *	@brief This API used to write the gyro sleep duration
 *	from page one register from 0x0D bit 0 to 2
 *
 *	@param v_sleep_durn_u8 : The value of gyro sleep duration
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_sleep_durn(u8 *v_sleep_durn_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel range is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			v_stat_s8 = bno055_write_page_id(PAGE_ONE);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ONE)) {
			/* Read the gyro sleep duration */
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_SLEEP_DURN__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_sleep_durn_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_GYRO_SLEEP_DURN);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to write the gyro sleep duration
 *	from page one register from 0x0D bit 0 to 2
 *
 *	@param v_sleep_durn_u8 : The value of gyro sleep duration
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_sleep_durn(u8 v_sleep_durn_u8)
{
BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 v_data_u8r = BNO055_ZERO_U8X;
s8 v_stat_s8 = ERROR;
s8 v_pg_stat_s8 = ERROR;
u8 v_prev_opmode_u8 = OPERATION_MODE_CONFIG;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
} else {
/* The write operation effective only if the operation
mode is in config mode, this part of code is checking the
current operation mode and set the config mode */
v_stat_s8 = bno055_get_operation_mode(&v_prev_opmode_u8);
if (v_stat_s8 == SUCCESS) {
	if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
		v_stat_s8 += bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);
		if (v_stat_s8 == SUCCESS) {
			/* Write page as one */
			v_pg_stat_s8 = bno055_write_page_id(PAGE_ONE);
			if (v_pg_stat_s8 == SUCCESS) {
				if (v_sleep_durn_u8 < BNO055_EIGHT_U8X) {
					com_rslt =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_GYRO_SLEEP_DURN__REG,
					&v_data_u8r, BNO055_ONE_U8X);
					if (com_rslt == SUCCESS) {
						/* Write the gyro
						sleep duration */
						v_data_u8r = BNO055_SET_BITSLICE
						(v_data_u8r,
						BNO055_GYRO_SLEEP_DURN,
						v_sleep_durn_u8);
						com_rslt +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_GYRO_SLEEP_DURN__REG,
						&v_data_u8r, BNO055_ONE_U8X);
					}
				} else {
				com_rslt = E_BNO055_OUT_OF_RANGE;
				}
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	} else {
	com_rslt = ERROR;
	}
}
if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
	/* set the operation mode of
	previous operation mode*/
	com_rslt += bno055_set_operation_mode
	(v_prev_opmode_u8);
return com_rslt;
}
/*!
 *	@brief This API used to read the gyro auto sleep duration
 *	from page one register from 0x0D bit 3 to 5
 *
 *	@param v_auto_sleep_durn_u8 : The value of gyro auto sleep duration
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_auto_sleep_durn(
u8 *v_auto_sleep_durn_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel range is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			v_stat_s8 = bno055_write_page_id(PAGE_ONE);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ONE)) {
			/* Read the value of gyro auto sleep duration */
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_AUTO_SLEEP_DURN__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_auto_sleep_durn_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_GYRO_AUTO_SLEEP_DURN);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to write the gyro auto sleep duration
 *	from page one register from 0x0D bit 3 to 5
 *
 *	@param v_auto_sleep_durn_u8 : The value of gyro auto sleep duration
 *	@param bw : The value of gyro bandwidth
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_gyro_set_auto_sleep_durn(
u8 v_auto_sleep_durn_u8, u8 bw)
{
BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 v_data_u8r = BNO055_ZERO_U8X;
s8 v_stat_s8 = ERROR;
u8 v_auto_sleep_durn_u8r;
u8 v_prev_opmode_u8 = OPERATION_MODE_CONFIG;
s8 v_pg_stat_s8 = ERROR;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
} else {
/* The write operation effective only if the operation
mode is in config mode, this part of code is checking the
current operation mode and set the config mode */
v_stat_s8 = bno055_get_operation_mode(&v_prev_opmode_u8);
if (v_stat_s8 == SUCCESS) {
	if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
		v_stat_s8 += bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);
	if (v_stat_s8 == SUCCESS) {
		/* Write page as one */
		v_pg_stat_s8 = bno055_write_page_id(PAGE_ONE);
		if (v_pg_stat_s8 == SUCCESS) {
			/* Write the value of gyro sleep duration */
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_AUTO_SLEEP_DURN__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			if (v_auto_sleep_durn_u8 < BNO055_EIGHT_U8X) {
				switch (bw) {
				case GYRO_BW_523HZ:
				if (v_auto_sleep_durn_u8 >
					BNO055_GYRO_4MS_AUTOSLPDUR)
					v_auto_sleep_durn_u8r =
					v_auto_sleep_durn_u8;
				else
					v_auto_sleep_durn_u8r =
					BNO055_GYRO_4MS_AUTOSLPDUR;
				break;
				case GYRO_BW_230HZ:
				if (v_auto_sleep_durn_u8 >
					BNO055_GYRO_4MS_AUTOSLPDUR)
					v_auto_sleep_durn_u8r =
					v_auto_sleep_durn_u8;
				else
					v_auto_sleep_durn_u8r =
					BNO055_GYRO_4MS_AUTOSLPDUR;
				break;
				case GYRO_BW_116HZ:
				if (v_auto_sleep_durn_u8 >
					BNO055_GYRO_4MS_AUTOSLPDUR)
					v_auto_sleep_durn_u8r =
					v_auto_sleep_durn_u8;
				else
					v_auto_sleep_durn_u8r =
					BNO055_GYRO_4MS_AUTOSLPDUR;
				break;
				case GYRO_BW_47HZ:
				if (v_auto_sleep_durn_u8 >
					BNO055_GYRO_5MS_AUTOSLPDUR)
					v_auto_sleep_durn_u8r =
					v_auto_sleep_durn_u8;
				else
					v_auto_sleep_durn_u8r =
					BNO055_GYRO_5MS_AUTOSLPDUR;
				break;
				case GYRO_BW_23HZ:
				if (v_auto_sleep_durn_u8 >
					BNO055_GYRO_10MS_AUTOSLPDUR)
					v_auto_sleep_durn_u8r =
					v_auto_sleep_durn_u8;
				else
					v_auto_sleep_durn_u8r =
					BNO055_GYRO_10MS_AUTOSLPDUR;
				break;
				case GYRO_BW_12HZ:
				if (v_auto_sleep_durn_u8 >
					BNO055_GYRO_20MS_AUTOSLPDUR)
					v_auto_sleep_durn_u8r =
					v_auto_sleep_durn_u8;
				else
					v_auto_sleep_durn_u8r =
					BNO055_GYRO_20MS_AUTOSLPDUR;
				break;
				case GYRO_BW_64HZ:
				if (v_auto_sleep_durn_u8 >
					BNO055_GYRO_10MS_AUTOSLPDUR)
					v_auto_sleep_durn_u8r =
					v_auto_sleep_durn_u8;
				else
					v_auto_sleep_durn_u8r =
					BNO055_GYRO_10MS_AUTOSLPDUR;
				break;
				case GYRO_BW_32HZ:
				if (v_auto_sleep_durn_u8 >
					BNO055_GYRO_20MS_AUTOSLPDUR)
					v_auto_sleep_durn_u8r =
					v_auto_sleep_durn_u8;
				else
					v_auto_sleep_durn_u8r =
					BNO055_GYRO_20MS_AUTOSLPDUR;
				break;
				default:
				if (v_auto_sleep_durn_u8 >
					BNO055_GYRO_4MS_AUTOSLPDUR)
					v_auto_sleep_durn_u8r =
					v_auto_sleep_durn_u8;
				else
					v_auto_sleep_durn_u8r =
					BNO055_GYRO_4MS_AUTOSLPDUR;
				break;
				}
				if (com_rslt == SUCCESS) {
					v_data_u8r = BNO055_SET_BITSLICE
					(v_data_u8r,
					BNO055_GYRO_AUTO_SLEEP_DURN,
					v_auto_sleep_durn_u8r);
					com_rslt +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_GYRO_AUTO_SLEEP_DURN__REG,
					&v_data_u8r, BNO055_ONE_U8X);
				}
			} else {
			com_rslt = E_BNO055_OUT_OF_RANGE;
			}
		} else {
		com_rslt = ERROR;
		}
	} else {
	com_rslt = ERROR;
	}
} else {
com_rslt = ERROR;
}
}
if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
	/* set the operation mode of
	previous operation mode*/
	com_rslt += bno055_set_operation_mode
	(v_prev_opmode_u8);
return com_rslt;
}
/*!
 *	@brief This API used to read the mag sleep mode
 *	from page one register from 0x0E bit 0
 *
 *	@param v_sleep_mode_u8 : The value of mag sleep mode
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_mag_sleep_mode(
u8 *v_sleep_mode_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page,mag sleep mode is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			v_stat_s8 = bno055_write_page_id(PAGE_ONE);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ONE)) {
			/* Read the value of mag sleep mode*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_MAG_SLEEP_MODE__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_sleep_mode_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_MAG_SLEEP_MODE);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to write the mag sleep mode
 *	from page one register from 0x0E bit 0
 *
 *	@param v_sleep_mode_u8 : The value of mag sleep mode
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_mag_sleep_mode(
u8 v_sleep_mode_u8)
{
BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 v_data_u8r = BNO055_ZERO_U8X;
s8 v_stat_s8 = ERROR;
s8 v_pg_stat_s8 = ERROR;
u8 v_prev_opmode_u8 = OPERATION_MODE_CONFIG;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
} else {
	/* The write operation effective only if the operation
	mode is in config mode, this part of code is checking the
	current operation mode and set the config mode */
	v_stat_s8 = bno055_get_operation_mode(&v_prev_opmode_u8);
	if (v_stat_s8 == SUCCESS) {
		if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
			v_stat_s8 += bno055_set_operation_mode
			(OPERATION_MODE_CONFIG);
			if (v_stat_s8 == SUCCESS) {
				/* Write page as one */
				v_pg_stat_s8 = bno055_write_page_id(PAGE_ONE);
				if (v_pg_stat_s8 == SUCCESS) {
					com_rslt =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_MAG_SLEEP_MODE__REG,
					&v_data_u8r, BNO055_ONE_U8X);
					if (com_rslt == SUCCESS) {
						/* Write the value
						of mag sleep mode*/
						v_data_u8r =
						BNO055_SET_BITSLICE(v_data_u8r,
						BNO055_MAG_SLEEP_MODE,
						v_sleep_mode_u8);
						com_rslt +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_MAG_SLEEP_MODE__REG,
						&v_data_u8r, BNO055_ONE_U8X);
					}
				} else {
				com_rslt = ERROR;
				}
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	}
	if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
		/* set the operation mode of
		previous operation mode*/
		com_rslt += bno055_set_operation_mode
		(v_prev_opmode_u8);
	return com_rslt;
}
/*!
 *	@brief This API used to read the mag sleep duration
 *	from page one register from 0x0E bit 1 to 4
 *
 *	@param v_sleep_durn_u8 : The value of mag sleep duration
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_mag_sleep_durn(
u8 *v_sleep_durn_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page,mag sleep duration is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			v_stat_s8 = bno055_write_page_id(PAGE_ONE);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ONE)) {
			/* Read the value of mag sleep duration*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_MAG_SLEEP_DURN__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_sleep_durn_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_MAG_SLEEP_DURN);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to write the mag sleep duration
 *	from page one register from 0x0E bit 1 to 4
 *
 *	@param v_sleep_durn_u8 : The value of mag sleep duration
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_mag_sleep_durn(
u8 v_sleep_durn_u8)
{
BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 v_data_u8r = BNO055_ZERO_U8X;
s8 v_stat_s8 = ERROR;
s8 v_pg_stat_s8 = ERROR;
u8 v_prev_opmode_u8 = OPERATION_MODE_CONFIG;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
} else {
	/* The write operation effective only if the operation
	mode is in config mode, this part of code is checking the
	current operation mode and set the config mode */
	v_stat_s8 = bno055_get_operation_mode(&v_prev_opmode_u8);
	if (v_stat_s8 == SUCCESS) {
		if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
			v_stat_s8 += bno055_set_operation_mode
			(OPERATION_MODE_CONFIG);
			if (v_stat_s8 == SUCCESS) {
				/* Write page as one */
				v_pg_stat_s8 = bno055_write_page_id(PAGE_ONE);
				if (v_pg_stat_s8 == SUCCESS) {
					com_rslt =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_MAG_SLEEP_DURN__REG,
					&v_data_u8r, BNO055_ONE_U8X);
					if (com_rslt == SUCCESS) {
						/* Write the value of
						mag sleep duration */
						v_data_u8r =
						BNO055_SET_BITSLICE(v_data_u8r,
						BNO055_MAG_SLEEP_DURN,
						v_sleep_durn_u8);
						com_rslt +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_MAG_SLEEP_DURN__REG,
						&v_data_u8r, BNO055_ONE_U8X);
					}
				} else {
				com_rslt = ERROR;
				}
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	}
	if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
		/* set the operation mode of
		previous operation mode*/
		com_rslt += bno055_set_operation_mode
		(v_prev_opmode_u8);
	return com_rslt;
}
/*!
 *	@brief This API used to read the gyro anymotion interrupt mask
 *	from page one register from 0x0F bit 2
 *
 *	@param v_gyro_any_motion_u8 : The value of gyro anymotion interrupt mask
 *		v_gyro_any_motion_u8 |   result
 *     --------------------  |------------
 *              0x01         | ENABLED
 *              0x00         | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note While enabling the gyro anymotion interrupt
 *	configure the following settings
 *
 *	Axis:
 *	bno055_set_gyro_any_motion_axis_enable()
 *
 *	Filter setting:
 *	bno055_set_gyro_any_motion_filter()
 *
 *	Threshold :
 *
 *	bno055_set_gyro_any_motion_thres()
 *
 *	Slope samples :
 *
 *	bno055_set_gyro_any_motion_slope_samples()
 *
 *	Awake duration :
 *
 *	bno055_set_gyro_any_motion_awake_durn()
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_intr_mask_gyro_any_motion(
u8 *v_gyro_any_motion_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, gyro anymotion interrupt mask is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			v_stat_s8 = bno055_write_page_id(PAGE_ONE);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ONE)) {
			/* Read the value of gyro anymotion interrupt mask*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_ANY_MOTION_INTR_MASK__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_gyro_any_motion_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_GYRO_ANY_MOTION_INTR_MASK);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to write the gyro anymotion interrupt mask
 *	from page one register from 0x0F bit 2
 *
 *	@param v_gyro_any_motion_u8 : The value of gyro anymotion interrupt mask
 *		v_gyro_any_motion_u8 |   result
 *     --------------------  |------------
 *              0x01         | ENABLED
 *              0x00         | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note While enabling the gyro anymotion interrupt
 *	configure the following settings
 *
 *	Axis:
 *	bno055_set_gyro_any_motion_axis_enable()
 *
 *	Filter setting:
 *	bno055_set_gyro_any_motion_filter()
 *
 *	Threshold :
 *
 *	bno055_set_gyro_any_motion_thres()
 *
 *	Slope samples :
 *
 *	bno055_set_gyro_any_motion_slope_samples()
 *
 *	Awake duration :
 *
 *	bno055_set_gyro_any_motion_awake_durn()
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_intr_mask_gyro_any_motion(
u8 v_gyro_any_motion_u8)
{
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel range is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			v_stat_s8 = bno055_write_page_id(PAGE_ONE);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ONE)) {
			/* Write the value of gyro anymotion interrupt mask*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_ANY_MOTION_INTR_MASK__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			if (com_rslt == SUCCESS) {
				v_data_u8r =
				BNO055_SET_BITSLICE(v_data_u8r,
				BNO055_GYRO_ANY_MOTION_INTR_MASK,
				v_gyro_any_motion_u8);
				com_rslt +=
				p_bno055->BNO055_BUS_WRITE_FUNC
				(p_bno055->dev_addr,
				BNO055_GYRO_ANY_MOTION_INTR_MASK__REG,
				&v_data_u8r, BNO055_ONE_U8X);
			}
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to read the gyro highrate interrupt mask
 *	from page one register from 0x0F bit 3
 *
 *	@param v_gyro_highrate_u8 : The value of gyro highrate interrupt mask
 *		  v_gyro_highrate_u8 |  result
 *     --------------------  |------------
 *              0x01         | ENABLED
 *              0x00         | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note While enabling the gyro highrate interrupt
 *	configure the below settings by using
 *	the following functions
 *
 *	Axis :
 *
 *	bno055_set_gyro_highrate_axis_enable()
 *
 *	Filter :
 *
 *	bno055_set_gyro_highrate_filter()
 *
 *	Threshold :
 *
 *	bno055_get_gyro_highrate_x_thres()
 *
 *	bno055_get_gyro_highrate_y_thres()
 *
 *	bno055_get_gyro_highrate_z_thres()
 *
 *	Hysteresis :
 *
 *	bno055_set_gyro_highrate_x_hyst()
 *
 *	bno055_set_gyro_highrate_y_hyst()
 *
 *	bno055_set_gyro_highrate_z_hyst()
 *
 *	Duration :
 *
 *	bno055_set_gyro_highrate_x_durn()
 *
 *	bno055_set_gyro_highrate_y_durn()
 *
 *	bno055_set_gyro_highrate_z_durn()
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_intr_mask_gyro_highrate(
u8 *v_gyro_highrate_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, gyro highrate interrupt mask is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			v_stat_s8 = bno055_write_page_id(PAGE_ONE);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ONE)) {
			/* Read the value of gyro highrate interrupt mask*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_HIGHRATE_INTR_MASK__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_gyro_highrate_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_GYRO_HIGHRATE_INTR_MASK);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to write the gyro highrate interrupt mask
 *	from page one register from 0x0F bit 3
 *
 *	@param v_gyro_highrate_u8 : The value of gyro highrate interrupt mask
 *		  v_gyro_highrate_u8 |  result
 *     --------------------  |------------
 *              0x01         | ENABLED
 *              0x00         | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note While enabling the gyro highrate interrupt
 *	configure the below settings by using
 *	the following functions
 *
 *	Axis :
 *
 *	bno055_set_gyro_highrate_axis_enable()
 *
 *	Filter :
 *
 *	bno055_set_gyro_highrate_filter()
 *
 *	Threshold :
 *
 *	bno055_get_gyro_highrate_x_thres()
 *
 *	bno055_get_gyro_highrate_y_thres()
 *
 *	bno055_get_gyro_highrate_z_thres()
 *
 *	Hysteresis :
 *
 *	bno055_set_gyro_highrate_x_hyst()
 *
 *	bno055_set_gyro_highrate_y_hyst()
 *
 *	bno055_set_gyro_highrate_z_hyst()
 *
 *	Duration :
 *
 *	bno055_set_gyro_highrate_x_durn()
 *
 *	bno055_set_gyro_highrate_y_durn()
 *
 *	bno055_set_gyro_highrate_z_durn()
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_intr_mask_gyro_highrate(
u8 v_gyro_highrate_u8)
{
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, gyro highrate interrupt mask is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			v_stat_s8 = bno055_write_page_id(PAGE_ONE);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ONE)) {
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_HIGHRATE_INTR_MASK__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			if (com_rslt == SUCCESS) {
				/* Write the value of gyro
				highrate interrupt mask*/
				v_data_u8r =
				BNO055_SET_BITSLICE(v_data_u8r,
				BNO055_GYRO_HIGHRATE_INTR_MASK,
				v_gyro_highrate_u8);
				com_rslt +=
				p_bno055->BNO055_BUS_WRITE_FUNC
				(p_bno055->dev_addr,
				BNO055_GYRO_HIGHRATE_INTR_MASK__REG,
				&v_data_u8r, BNO055_ONE_U8X);
			}
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to read the accel highg interrupt mask
 *	from page one register from 0x0F bit 5
 *
 *	@param v_accel_high_g_u8 : The value of accel highg interrupt mask
 *		   v_accel_high_g_u8 |   result
 *     --------------------  |------------
 *              0x01         | ENABLED
 *              0x00         | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note While enabling the accel highg interrupt
 *	configure the below settings by using
 *	the following functions
 *
 *	Axis :
 *
 *	bno055_set_accel_high_g_axis_enable()
 *
 *	Threshold :
 *
 *	bno055_set_accel_high_g_thres()
 *
 *	Duration :
 *
 *	bno055_set_accel_high_g_durn()
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_intr_mask_accel_high_g(
u8 *v_accel_high_g_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel highg interrupt mask is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			v_stat_s8 = bno055_write_page_id(PAGE_ONE);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ONE)) {
			/* Read the value of accel highg interrupt mask*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_HIGH_G_INTR_MASK__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_accel_high_g_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_ACCEL_HIGH_G_INTR_MASK);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to write the accel highg interrupt mask
 *	from page one register from 0x0F bit 5
 *
 *	@param v_accel_high_g_u8 : The value of accel highg interrupt mask
 *		   v_accel_high_g_u8 |   result
 *     --------------------  |------------
 *              0x01         | ENABLED
 *              0x00         | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note While enabling the accel highg interrupt
 *	configure the below settings by using
 *	the following functions
 *
 *	Axis :
 *
 *	bno055_set_accel_high_g_axis_enable()
 *
 *	Threshold :
 *
 *	bno055_set_accel_high_g_thres()
 *
 *	Duration :
 *
 *	bno055_set_accel_high_g_durn()
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_intr_mask_accel_high_g(
u8 v_accel_high_g_u8)
{
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel highg interrupt mask is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			v_stat_s8 = bno055_write_page_id(PAGE_ONE);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ONE)) {
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_HIGH_G_INTR_MASK__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			if (com_rslt == SUCCESS) {
				/* Write the value of accel
				highg interrupt mask*/
				v_data_u8r =
				BNO055_SET_BITSLICE(v_data_u8r,
				BNO055_ACCEL_HIGH_G_INTR_MASK,
				v_accel_high_g_u8);
				com_rslt +=
				p_bno055->BNO055_BUS_WRITE_FUNC
				(p_bno055->dev_addr,
				BNO055_ACCEL_HIGH_G_INTR_MASK__REG,
				&v_data_u8r, BNO055_ONE_U8X);
			}
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to read the accel anymotion interrupt mask
 *	from page one register from 0x0F bit 6
 *
 *	@param v_accel_any_motion_u8 : The value of accel anymotion interrupt mask
 *     v_accel_any_motion_u8 | result
 *     --------------------  |------------
 *              0x01         | ENABLED
 *              0x00         | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note While enabling the accel highg interrupt
 *	configure the below settings by using
 *	the following functions
 *
 *	Axis :
 *
 *	bno055_set_accel_high_g_axis_enable()
 *
 *	Threshold :
 *
 *	bno055_set_accel_high_g_thres()
 *
 *	Duration :
 *
 *	bno055_set_accel_high_g_durn()
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_intr_mask_accel_any_motion(
u8 *v_accel_any_motion_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel anymotion interrupt mask is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			v_stat_s8 = bno055_write_page_id(PAGE_ONE);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ONE)) {
			/* The value of accel anymotion interrupt mask*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_ANY_MOTION_INTR_MASK__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_accel_any_motion_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_ACCEL_ANY_MOTION_INTR_MASK);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to write the accel anymotion interrupt mask
 *	from page one register from 0x0F bit 6
 *
 *	@param v_accel_any_motion_u8 : The value of accel anymotion interrupt mask
 *     v_accel_any_motion_u8 | result
 *     --------------------  |------------
 *              0x01         | ENABLED
 *              0x00         | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note While enabling the accel anymotion interrupt
 *	configure the following settings
 *
 *	Axis:
 *
 *	bno055_set_accel_any_motion_no_motion_axis_enable()
 *
 *	Duration:
 *
 *	bno055_set_accel_any_motion_durn()
 *
 * Threshold:
 *
 *	bno055_set_accel_any_motion_thres()
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_intr_mask_accel_any_motion(
u8 v_accel_any_motion_u8)
{
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel anymotion interrupt mask is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			v_stat_s8 = bno055_write_page_id(PAGE_ONE);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ONE)) {
			/* Write the value of accel anymotion interrupt mask*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_ANY_MOTION_INTR_MASK__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			if (com_rslt == SUCCESS) {
				v_data_u8r =
				BNO055_SET_BITSLICE(v_data_u8r,
				BNO055_ACCEL_ANY_MOTION_INTR_MASK,
				v_accel_any_motion_u8);
				com_rslt +=
				p_bno055->BNO055_BUS_WRITE_FUNC
				(p_bno055->dev_addr,
				BNO055_ACCEL_ANY_MOTION_INTR_MASK__REG,
				&v_data_u8r, BNO055_ONE_U8X);
			}
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to read the accel nomotion interrupt mask
 *	from page one register from 0x0F bit 7
 *
 *	@param v_accel_nomotion_u8 : The value of accel nomotion interrupt mask
 *     v_accel_nomotion_u8   | result
 *     --------------------  |------------
 *              0x01         | ENABLED
 *              0x00         | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *
 *	@note While enabling the accel anymotion interrupt
 *	configure the following settings
 *
 *	Axis:
 *
 *	bno055_set_accel_any_motion_no_motion_axis_enable()
 *
 *	Duration:
 *
 *	bno055_set_accel_any_motion_durn()
 *
 * Threshold:
 *
 *	bno055_set_accel_any_motion_thres())
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_intr_mask_accel_no_motion(
u8 *v_accel_nomotion_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel nomotion interrupt mask is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			v_stat_s8 = bno055_write_page_id(PAGE_ONE);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ONE)) {
			/* Read the value of accel nomotion interrupt mask*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_NO_MOTION_INTR_MASK__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_accel_nomotion_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_ACCEL_NO_MOTION_INTR_MASK);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to write the accel nomotion interrupt mask
 *	from page one register from 0x0F bit 7
 *
 *	@param v_accel_nomotion_u8 : The value of accel nomotion interrupt mask
 *     v_accel_nomotion_u8   | result
 *     --------------------  |------------
 *              0x01         | ENABLED
 *              0x00         | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note While enabling the accel nomotion interrupt
 *	configure the following settings
 *
 *	Axis:
 *
 *	bno055_set_accel_any_motion_no_motion_axis_enable()
 *
 *	Threshold :
 *
 *	bno055_set_accel_slow_no_motion_thres()
 *
 *	Duration :
 *
 *	bno055_set_accel_slow_no_motion_durn()
 *
 *	Slow/no motion enable:
 *
 *	bno055_set_accel_slow_no_motion_enable()
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_intr_mask_accel_no_motion(
u8 v_accel_nomotion_u8)
{
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel
		nomotion interrupt mask is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			v_stat_s8 = bno055_write_page_id(PAGE_ONE);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ONE)) {
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_NO_MOTION_INTR_MASK__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			if (com_rslt == SUCCESS) {
				/* Write the value of accel
				nomotion interrupt mask*/
				v_data_u8r =
				BNO055_SET_BITSLICE(v_data_u8r,
				BNO055_ACCEL_NO_MOTION_INTR_MASK,
				v_accel_nomotion_u8);
				com_rslt +=
				p_bno055->BNO055_BUS_WRITE_FUNC
				(p_bno055->dev_addr,
				BNO055_ACCEL_NO_MOTION_INTR_MASK__REG,
				&v_data_u8r, BNO055_ONE_U8X);
			}
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to read the gyro anymotion interrupt
 *	from page one register from 0x10 bit 2
 *
 *	@param v_gyro_any_motion_u8 : The value of gyro anymotion interrupt
 *		v_gyro_any_motion_u8 | result
 *     --------------------  |------------
 *              0x01         | ENABLED
 *              0x00         | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note While enabling the gyro anymotion interrupt
 *	configure the following settings
 *
 *	Axis:
 *	bno055_set_gyro_any_motion_axis_enable()
 *
 *	Filter setting:
 *	bno055_set_gyro_any_motion_filter()
 *
 *	Threshold :
 *
 *	bno055_set_gyro_any_motion_thres()
 *
 *	Slope samples :
 *
 *	bno055_set_gyro_any_motion_slope_samples()
 *
 *	Awake duration :
 *
 *	bno055_set_gyro_any_motion_awake_durn()
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_intr_gyro_any_motion(
u8 *v_gyro_any_motion_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, gyro anymotion interrupt  is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			v_stat_s8 = bno055_write_page_id(PAGE_ONE);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ONE)) {
			/* Read the value of gyro anymotion interrupt */
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_ANY_MOTION_INTR__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_gyro_any_motion_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_GYRO_ANY_MOTION_INTR);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to write the gyro anymotion interrupt
 *	from page one register from 0x10 bit 2
 *
 *	@param v_gyro_any_motion_u8 : The value of gyro anymotion interrupt
 *	  v_gyro_any_motion_u8   | result
 *     --------------------  |------------
 *              0x01         | ENABLED
 *              0x00         | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note While enabling the gyro anymotion interrupt
 *	configure the following settings
 *
 *	Axis:
 *	bno055_set_gyro_any_motion_axis_enable()
 *
 *	Filter setting:
 *	bno055_set_gyro_any_motion_filter()
 *
 *	Threshold :
 *
 *	bno055_set_gyro_any_motion_thres()
 *
 *	Slope samples :
 *
 *	bno055_set_gyro_any_motion_slope_samples()
 *
 *	Awake duration :
 *
 *	bno055_set_gyro_any_motion_awake_durn()
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_intr_gyro_any_motion(
u8 v_gyro_any_motion_u8)
{
BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 v_data_u8r = BNO055_ZERO_U8X;
s8 v_stat_s8 = ERROR;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
	} else {
	/*condition check for page, gyro anymotion interrupt  is
	available in the page one*/
	if (p_bno055->page_id != PAGE_ONE)
		/* Write page as one */
		v_stat_s8 = bno055_write_page_id(PAGE_ONE);
	if ((v_stat_s8 == SUCCESS) ||
	(p_bno055->page_id == PAGE_ONE)) {
		/* Write the value of gyro anymotion interrupt */
		com_rslt = p_bno055->BNO055_BUS_READ_FUNC
		(p_bno055->dev_addr,
		BNO055_GYRO_ANY_MOTION_INTR__REG,
		&v_data_u8r, BNO055_ONE_U8X);
		if (com_rslt == SUCCESS) {
			v_data_u8r =
			BNO055_SET_BITSLICE(v_data_u8r,
			BNO055_GYRO_ANY_MOTION_INTR,
			v_gyro_any_motion_u8);
			com_rslt +=
			p_bno055->BNO055_BUS_WRITE_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_ANY_MOTION_INTR__REG,
			&v_data_u8r, BNO055_ONE_U8X);
		}
	} else {
	com_rslt = ERROR;
	}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to read the gyro highrate interrupt
 *	from page one register from 0x10 bit 3
 *
 *	@param v_gyro_highrate_u8 : The value of gyro highrate interrupt
 *		v_gyro_highrate_u8   | result
 *     --------------------  |------------
 *              0x01         | ENABLED
 *              0x00         | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note While enabling the gyro highrate interrupt
 *	configure the below settings by using
 *	the following functions
 *
 *	Axis :
 *
 *	bno055_set_gyro_highrate_axis_enable()
 *
 *	Filter :
 *
 *	bno055_set_gyro_highrate_filter()
 *
 *	Threshold :
 *
 *	bno055_get_gyro_highrate_x_thres()
 *
 *	bno055_get_gyro_highrate_y_thres()
 *
 *	bno055_get_gyro_highrate_z_thres()
 *
 *	Hysteresis :
 *
 *	bno055_set_gyro_highrate_x_hyst()
 *
 *	bno055_set_gyro_highrate_y_hyst()
 *
 *	bno055_set_gyro_highrate_z_hyst()
 *
 *	Duration :
 *
 *	bno055_set_gyro_highrate_x_durn()
 *
 *	bno055_set_gyro_highrate_y_durn()
 *
 *	bno055_set_gyro_highrate_z_durn()
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_intr_gyro_highrate(
u8 *v_gyro_highrate_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, gyro highrate interrupt is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			v_stat_s8 = bno055_write_page_id(PAGE_ONE);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ONE)) {
			/* Read the value of gyro highrate interrupt */
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_HIGHRATE_INTR__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_gyro_highrate_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_GYRO_HIGHRATE_INTR);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to write the gyro highrate interrupt
 *	from page one register from 0x10 bit 3
 *
 *	@param v_gyro_highrate_u8 : The value of gyro highrate interrupt
 *		v_gyro_highrate_u8   | result
 *     --------------------  |------------
 *              0x01         | ENABLED
 *              0x00         | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note While enabling the gyro highrate interrupt
 *	configure the below settings by using
 *	the following functions
 *
 *	Axis :
 *
 *	bno055_set_gyro_highrate_axis_enable()
 *
 *	Filter :
 *
 *	bno055_set_gyro_highrate_filter()
 *
 *	Threshold :
 *
 *	bno055_get_gyro_highrate_x_thres()
 *
 *	bno055_get_gyro_highrate_y_thres()
 *
 *	bno055_get_gyro_highrate_z_thres()
 *
 *	Hysteresis :
 *
 *	bno055_set_gyro_highrate_x_hyst()
 *
 *	bno055_set_gyro_highrate_y_hyst()
 *
 *	bno055_set_gyro_highrate_z_hyst()
 *
 *	Duration :
 *
 *	bno055_set_gyro_highrate_x_durn()
 *
 *	bno055_set_gyro_highrate_y_durn()
 *
 *	bno055_set_gyro_highrate_z_durn()
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_intr_gyro_highrate(
u8 v_gyro_highrate_u8)
{
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, gyro highrate interrupt is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			v_stat_s8 = bno055_write_page_id(PAGE_ONE);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ONE)) {
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_HIGHRATE_INTR__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			if (com_rslt == SUCCESS) {
				/* Write the value of gyro highrate interrupt */
				v_data_u8r =
				BNO055_SET_BITSLICE(v_data_u8r,
				BNO055_GYRO_HIGHRATE_INTR, v_gyro_highrate_u8);
				com_rslt +=
				p_bno055->BNO055_BUS_WRITE_FUNC
				(p_bno055->dev_addr,
				BNO055_GYRO_HIGHRATE_INTR__REG,
				&v_data_u8r, BNO055_ONE_U8X);
			}
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to read the accel highg interrupt
 *	from page one register from 0x10 bit 5
 *
 *	@param v_accel_high_g_u8 : The value of accel highg interrupt
 *		v_accel_high_g_u8    | result
 *     --------------------  |------------
 *              0x01         | ENABLED
 *              0x00         | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note While enabling the accel highg interrupt
 *	configure the below settings by using
 *	the following functions
 *
 *	Axis :
 *
 *	bno055_set_accel_high_g_axis_enable()
 *
 *	Threshold :
 *
 *	bno055_set_accel_high_g_thres()
 *
 *	Duration :
 *
 *	bno055_set_accel_high_g_durn()
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_intr_accel_high_g(
u8 *v_accel_high_g_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel highg interrupt  is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			v_stat_s8 = bno055_write_page_id(PAGE_ONE);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ONE)) {
			/* Read the value of accel highg interrupt*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_HIGH_G_INTR__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_accel_high_g_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_ACCEL_HIGH_G_INTR);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to write the accel highg interrupt
 *	from page one register from 0x10 bit 5
 *
 *	@param v_accel_high_g_u8 : The value of accel highg interrupt
 *		v_accel_high_g_u8    | result
 *     --------------------  |------------
 *              0x01         | ENABLED
 *              0x00         | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note While enabling the accel highg interrupt
 *	configure the below settings by using
 *	the following functions
 *
 *	Axis :
 *
 *	bno055_set_accel_high_g_axis_enable()
 *
 *	Threshold :
 *
 *	bno055_set_accel_high_g_thres()
 *
 *	Duration :
 *
 *	bno055_set_accel_high_g_durn()
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_intr_accel_high_g(
u8 v_accel_high_g_u8)
{
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel highg interrupt is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			v_stat_s8 = bno055_write_page_id(PAGE_ONE);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ONE)) {
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_HIGH_G_INTR__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			if (com_rslt == SUCCESS) {
				/* Write the value of accel highg interrupt*/
				v_data_u8r =
				BNO055_SET_BITSLICE(v_data_u8r,
				BNO055_ACCEL_HIGH_G_INTR,
				v_accel_high_g_u8);
				com_rslt +=
				p_bno055->BNO055_BUS_WRITE_FUNC
				(p_bno055->dev_addr,
				BNO055_ACCEL_HIGH_G_INTR__REG,
				&v_data_u8r, BNO055_ONE_U8X);
			}
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to read the accel anymotion interrupt
 *	from page one register from 0x10 bit 6
 *
 *	@param v_accel_any_motion_u8 : The value of accel anymotion interrupt
 *	v_accel_any_motion_u8    | result
 *     --------------------  |------------
 *              0x01         | ENABLED
 *              0x00         | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note While enabling the accel anymotion interrupt
 *	configure the following settings
 *
 *	Axis:
 *
 *	bno055_set_accel_any_motion_no_motion_axis_enable()
 *
 *	Duration:
 *
 *	bno055_set_accel_any_motion_durn()
 *
 * Threshold:
 *
 *	bno055_set_accel_any_motion_thres()
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_intr_accel_any_motion(
u8 *v_accel_any_motion_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel anymotion interrupt  is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			v_stat_s8 = bno055_write_page_id(PAGE_ONE);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ONE)) {
			/* Read the value of accel anymotion interrupt */
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_ANY_MOTION_INTR__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_accel_any_motion_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_ACCEL_ANY_MOTION_INTR);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to write the accel anymotion interrupt
 *	from page one register from 0x10 bit 6
 *
 *	@param v_accel_any_motion_u8 : The value of accel anymotion interrupt
 *	v_accel_any_motion_u8    | result
 *     --------------------  |------------
 *              0x01         | ENABLED
 *              0x00         | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note While enabling the accel anymotion interrupt
 *	configure the following settings
 *
 *	Axis:
 *
 *	bno055_set_accel_any_motion_no_motion_axis_enable()
 *
 *	Duration:
 *
 *	bno055_set_accel_any_motion_durn()
 *
 *	Threshold:
 *
 *	bno055_set_accel_any_motion_thres()
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_intr_accel_any_motion(
u8 v_accel_any_motion_u8)
{
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel range is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			v_stat_s8 = bno055_write_page_id(PAGE_ONE);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ONE)) {
			/* Write the value of accel anymotion interrupt */
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_ANY_MOTION_INTR__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			if (com_rslt == SUCCESS) {
				v_data_u8r =
				BNO055_SET_BITSLICE(v_data_u8r,
				BNO055_ACCEL_ANY_MOTION_INTR,
				v_accel_any_motion_u8);
				com_rslt +=
				p_bno055->BNO055_BUS_WRITE_FUNC
				(p_bno055->dev_addr,
				BNO055_ACCEL_ANY_MOTION_INTR__REG,
				&v_data_u8r, BNO055_ONE_U8X);
			}
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to read the accel nomotion interrupt
 *	from page one register from 0x10 bit 6
 *
 *	@param v_accel_nomotion_u8 : The value of accel nomotion interrupt
 *	  v_accel_nomotion_u8    | result
 *     --------------------  |------------
 *              0x01         | ENABLED
 *              0x00         | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note While enabling the accel nomotion interrupt
 *	configure the following settings
 *
 *	Axis:
 *
 *	bno055_set_accel_any_motion_no_motion_axis_enable()
 *
 *	Threshold :
 *
 *	bno055_set_accel_slow_no_motion_thres()
 *
 *	Duration :
 *
 *	bno055_set_accel_slow_no_motion_durn()
 *
 *	Slow/no motion enable:
 *
 *	bno055_set_accel_slow_no_motion_enable()
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_intr_accel_no_motion(
u8 *v_accel_nomotion_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel nomotion interrupt is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			v_stat_s8 = bno055_write_page_id(PAGE_ONE);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ONE)) {
			/* Read the value of accel nomotion interrupt*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_NO_MOTION_INTR__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_accel_nomotion_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_ACCEL_NO_MOTION_INTR);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to write the accel nomotion interrupt
 *	from page one register from 0x10 bit 6
 *
 *	@param v_accel_nomotion_u8 : The value of accel nomotion interrupt
 *	  v_accel_nomotion_u8    | result
 *     --------------------  |------------
 *              0x01         | ENABLED
 *              0x00         | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note While enabling the accel nomotion interrupt
 *	configure the following settings
 *
 *	Axis:
 *
 *	bno055_set_accel_any_motion_no_motion_axis_enable()
 *
 *	Threshold :
 *
 *	bno055_set_accel_slow_no_motion_thres()
 *
 *	Duration :
 *
 *	bno055_set_accel_slow_no_motion_durn()
 *
 *	Slow/no motion enable:
 *
 *	bno055_set_accel_slow_no_motion_enable()
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_intr_accel_no_motion(
u8 v_accel_nomotion_u8)
{
BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 v_data_u8r = BNO055_ZERO_U8X;
s8 v_stat_s8 = ERROR;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
	} else {
	/*condition check for page,
	accel nomotion interrupt is
	available in the page one*/
	if (p_bno055->page_id != PAGE_ONE)
		/* Write page as one */
		v_stat_s8 = bno055_write_page_id(PAGE_ONE);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ONE)) {
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_NO_MOTION_INTR__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			if (com_rslt == SUCCESS) {
				/* Write the value of
				accel nomotion interrupt */
				v_data_u8r = BNO055_SET_BITSLICE(v_data_u8r,
				BNO055_ACCEL_NO_MOTION_INTR,
				v_accel_nomotion_u8);
				com_rslt += p_bno055->BNO055_BUS_WRITE_FUNC
				(p_bno055->dev_addr,
				BNO055_ACCEL_NO_MOTION_INTR__REG,
				&v_data_u8r, BNO055_ONE_U8X);
			}
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to read the accel any motion threshold
 *	from page one register from 0x11 bit 0 to 7
 *
 *	@param v_accel_any_motion_thres_u8 : The value of any motion threshold
 *  v_accel_any_motion_thres_u8 | result
 *  ------------------------    | -------------
 *              0x01            | ENABLED
 *              0x00            | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note Accel anymotion threshold dependent on the
 *	range values
 *
 *  v_accel_range_u8 |	threshold |	LSB
 * ------------- | ------------- | ---------
 *     2g        |    3.19mg     |   1LSB
 *     4g        |    7.81mg     |   1LSB
 *     8g        |    15.63mg    |   1LSB
 *     16g       |    31.25mg    |   1LSB
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_any_motion_thres(
u8 *v_accel_any_motion_thres_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel any motion threshold  is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			v_stat_s8 = bno055_write_page_id(PAGE_ONE);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ONE)) {
			/* Read the value of accel any motion threshold */
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_ANY_MOTION_THRES__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_accel_any_motion_thres_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_ACCEL_ANY_MOTION_THRES);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to write the accel any motion threshold
 *	from page one register from 0x11 bit 0 to 7
 *
 *	@param v_accel_any_motion_thres_u8 : The value of any motion threshold
 *  v_accel_any_motion_thres_u8 | result
 *  ------------------------    | -------------
 *              0x01            | ENABLED
 *              0x00            | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note Accel anymotion threshold dependent on the
 *	range values
 *
 *  v_accel_range_u8	 |	threshold	 |	LSB
 * ------------- | ------------- | ---------
 *     2g        |    3.19mg     |   1LSB
 *     4g        |    7.81mg     |   1LSB
 *     8g        |    15.63mg    |   1LSB
 *     16g       |    31.25mg    |   1LSB
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_any_motion_thres(
u8 v_accel_any_motion_thres_u8)
{
BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 v_data_u8r = BNO055_ZERO_U8X;
s8 v_stat_s8 = ERROR;
s8 v_pg_stat_s8 = ERROR;
u8 v_prev_opmode_u8 = OPERATION_MODE_CONFIG;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
} else {
/* The write operation effective only if the operation
mode is in config mode, this part of code is checking the
current operation mode and set the config mode */
v_stat_s8 = bno055_get_operation_mode(&v_prev_opmode_u8);
if (v_stat_s8 == SUCCESS) {
	if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
		v_stat_s8 += bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);
		if (v_stat_s8 == SUCCESS) {
			/* Write page as one */
			v_pg_stat_s8 = bno055_write_page_id(PAGE_ONE);
			if (v_pg_stat_s8 == SUCCESS) {
				com_rslt = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_ACCEL_ANY_MOTION_THRES__REG,
				&v_data_u8r, BNO055_ONE_U8X);
				if (com_rslt == SUCCESS) {
					/* Write the value of
					accel any motion threshold*/
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_ACCEL_ANY_MOTION_THRES,
					v_accel_any_motion_thres_u8);
					com_rslt +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_ACCEL_ANY_MOTION_THRES__REG,
					&v_data_u8r, BNO055_ONE_U8X);
				}
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	} else {
	com_rslt = ERROR;
	}
}
if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
	/* set the operation mode of
	previous operation mode*/
	com_rslt += bno055_set_operation_mode
	(v_prev_opmode_u8);
return com_rslt;
}
/*!
 *	@brief This API used to read the accel anymotion duration
 *	from page one register from 0x12 bit 0 to 1
 *
 *	@param v_accel_any_motion_durn_u8 : The value of accel anymotion duration
 * v_accel_any_motion_durn_u8  | result
 *  -------------------------  | -------------
 *              0x01           | ENABLED
 *              0x00           | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_any_motion_durn(
u8 *v_accel_any_motion_durn_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel anymotion duration  is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			v_stat_s8 = bno055_write_page_id(PAGE_ONE);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ONE)) {
			/* Read the value of accel anymotion duration */
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_ANY_MOTION_DURN_SET__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_accel_any_motion_durn_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_ACCEL_ANY_MOTION_DURN_SET);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to write the accel anymotion duration
 *	from page one register from 0x12 bit 0 to 1
 *
 *	@param v_accel_any_motion_durn_u8 : The value of accel anymotion duration
 *
 * v_accel_any_motion_durn_u8  | result
 *  -------------------------  | -------------
 *              0x01           | ENABLED
 *              0x00           | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_any_motion_durn(
u8 v_accel_any_motion_durn_u8)
{
BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 v_data_u8r = BNO055_ZERO_U8X;
s8 v_stat_s8 = ERROR;
s8 v_pg_stat_s8 = ERROR;
u8 v_prev_opmode_u8 = OPERATION_MODE_CONFIG;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
} else {
/* The write operation effective only if the operation
mode is in config mode, this part of code is checking the
current operation mode and set the config mode */
v_stat_s8 = bno055_get_operation_mode(&v_prev_opmode_u8);
if (v_stat_s8 == SUCCESS) {
	if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
		v_stat_s8 += bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);
		if (v_stat_s8 == SUCCESS) {
			/* Write page as one */
			v_pg_stat_s8 = bno055_write_page_id(PAGE_ONE);
			if (v_pg_stat_s8 == SUCCESS) {
				com_rslt = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_ACCEL_ANY_MOTION_DURN_SET__REG,
				&v_data_u8r, BNO055_ONE_U8X);
				if (com_rslt == SUCCESS) {
					/* Write the value of
					accel anymotion duration*/
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_ACCEL_ANY_MOTION_DURN_SET,
					v_accel_any_motion_durn_u8);
					com_rslt +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_ACCEL_ANY_MOTION_DURN_SET__REG,
					&v_data_u8r, BNO055_ONE_U8X);
				}
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	} else {
	com_rslt = ERROR;
	}
}
if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
	/* set the operation mode of
	previous operation mode*/
	com_rslt += bno055_set_operation_mode
	(v_prev_opmode_u8);
return com_rslt;
}
/*!
 *	@brief This API used to read the accel anymotion enable
 *	from page one register from 0x12 bit 2 to 4
 *
 *	@param v_data_u8 : The value of accel anymotion enable
 *	   v_data_u8 | result
 *  ------------ | -------------
 *      0x01     | ENABLED
 *      0x00     | DISABLED
 *	@param v_channel_u8 : The value of accel anymotion axis selection
 *           v_channel_u8                        | value
 *     --------------------------                | ----------
 *     BNO055_ACCEL_ANY_MOTION_NO_MOTION_X_AXIS  |   0
 *     BNO055_ACCEL_ANY_MOTION_NO_MOTION_Y_AXIS  |   1
 *     BNO055_ACCEL_ANY_MOTION_NO_MOTION_Y_AXIS  |   2
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_any_motion_no_motion_axis_enable(
u8 v_channel_u8, u8 *v_data_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel anymotion enable is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			v_stat_s8 = bno055_write_page_id(PAGE_ONE);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ONE)) {
			switch (v_channel_u8) {
			case BNO055_ACCEL_ANY_MOTION_NO_MOTION_X_AXIS:
				/* Read the value of accel anymotion x enable*/
				com_rslt = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_ACCEL_ANY_MOTION_X_AXIS__REG,
				&v_data_u8r, BNO055_ONE_U8X);
				*v_data_u8 =
				BNO055_GET_BITSLICE(v_data_u8r,
				BNO055_ACCEL_ANY_MOTION_X_AXIS);
				break;
			case BNO055_ACCEL_ANY_MOTION_NO_MOTION_Y_AXIS:
				/* Read the value of accel anymotion y enable*/
				com_rslt = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_ACCEL_ANY_MOTION_Y_AXIS__REG,
				&v_data_u8r, BNO055_ONE_U8X);
				*v_data_u8 =
				BNO055_GET_BITSLICE(v_data_u8r,
				BNO055_ACCEL_ANY_MOTION_Y_AXIS);
				break;
			case BNO055_ACCEL_ANY_MOTION_NO_MOTION_Z_AXIS:
				/* Read the value of accel anymotion z enable*/
				com_rslt = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_ACCEL_ANY_MOTION_Z_AXIS__REG,
				&v_data_u8r, BNO055_ONE_U8X);
				*v_data_u8 =
				BNO055_GET_BITSLICE(v_data_u8r,
				BNO055_ACCEL_ANY_MOTION_Z_AXIS);
				break;
			default:
				com_rslt = E_BNO055_OUT_OF_RANGE;
				break;
			}
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to write the accel anymotion enable
 *	from page one register from 0x12 bit 2 to 4
 *
 *	@param v_data_u8 : The value of accel anymotion enable
 *	   v_data_u8 | result
 *  ------------ | -------------
 *      0x01     | ENABLED
 *      0x00     | DISABLED
 *	@param v_channel_u8 : The value of accel anymotion axis selection
 *           v_channel_u8                        | value
 *     --------------------------                | ----------
 *     BNO055_ACCEL_ANY_MOTION_NO_MOTION_X_AXIS  |   0
 *     BNO055_ACCEL_ANY_MOTION_NO_MOTION_Y_AXIS  |   1
 *     BNO055_ACCEL_ANY_MOTION_NO_MOTION_Y_AXIS  |   2
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_any_motion_no_motion_axis_enable(
u8 v_channel_u8, u8 v_data_u8)
{
BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 v_data_u8r = BNO055_ZERO_U8X;
s8 v_stat_s8 = ERROR;
s8 v_pg_stat_s8 = ERROR;
u8 v_prev_opmode_u8 = OPERATION_MODE_CONFIG;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
} else {
/* The write operation effective only if the operation
mode is in config mode, this part of code is checking the
current operation mode and set the config mode */
v_stat_s8 = bno055_get_operation_mode(&v_prev_opmode_u8);
if (v_stat_s8 == SUCCESS) {
	if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
		v_stat_s8 += bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);
		if (v_stat_s8 == SUCCESS) {
			/* Write page as one */
			v_pg_stat_s8 = bno055_write_page_id(PAGE_ONE);
			if (v_pg_stat_s8 == SUCCESS) {
				switch (v_channel_u8) {
				case BNO055_ACCEL_ANY_MOTION_NO_MOTION_X_AXIS:
				/* Write the value of
				accel anymotion x enable*/
				com_rslt = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_ACCEL_ANY_MOTION_X_AXIS__REG,
				&v_data_u8r, BNO055_ONE_U8X);
				if (com_rslt == SUCCESS) {
					v_data_u8r = BNO055_SET_BITSLICE
					(v_data_u8r,
					BNO055_ACCEL_ANY_MOTION_X_AXIS,
					v_data_u8);
					com_rslt +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_ACCEL_ANY_MOTION_X_AXIS__REG,
					&v_data_u8r, BNO055_ONE_U8X);
				}
				break;
				case BNO055_ACCEL_ANY_MOTION_NO_MOTION_Y_AXIS:
				/* Write the value of
				accel anymotion y enable*/
				com_rslt = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_ACCEL_ANY_MOTION_Y_AXIS__REG,
				&v_data_u8r, BNO055_ONE_U8X);
				if (com_rslt == SUCCESS) {
					v_data_u8r = BNO055_SET_BITSLICE
					(v_data_u8r,
					BNO055_ACCEL_ANY_MOTION_Y_AXIS,
					v_data_u8);
					com_rslt +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_ACCEL_ANY_MOTION_Y_AXIS__REG,
					&v_data_u8r, BNO055_ONE_U8X);
				}
				break;
				case BNO055_ACCEL_ANY_MOTION_NO_MOTION_Z_AXIS:
				/* Write the value of
				accel anymotion z enable*/
				com_rslt = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_ACCEL_ANY_MOTION_Z_AXIS__REG,
				&v_data_u8r, BNO055_ONE_U8X);
				if (com_rslt == SUCCESS) {
					v_data_u8r = BNO055_SET_BITSLICE
					(v_data_u8r,
					BNO055_ACCEL_ANY_MOTION_Z_AXIS,
					v_data_u8);
					com_rslt +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_ACCEL_ANY_MOTION_Z_AXIS__REG,
					&v_data_u8r, BNO055_ONE_U8X);
				}
				break;
				default:
				com_rslt = E_BNO055_OUT_OF_RANGE;
				break;
				}
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	} else {
	com_rslt = ERROR;
	}
}
if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
	/* set the operation mode of
	previous operation mode*/
	com_rslt += bno055_set_operation_mode
	(v_prev_opmode_u8);
return com_rslt;
}
/*!
 *	@brief This API used to read the accel highg enable
 *	from page one register from 0x12 bit 5 to 7
 *
 *	@param v_data_u8 : The value of accel highg enable
 *      v_data_u8| result
 *  ------------ | -------------
 *      0x01     | ENABLED
 *      0x00     | DISABLED
 *	@param v_channel_u8 : The value of accel highg axis selection
 *               v_channel_u8     | value
 *     -------------------------- | ----------
 *     BNO055_ACCEL_HIGH_G_X_AXIS |   0
 *     BNO055_ACCEL_HIGH_G_Y_AXIS |   1
 *     BNO055_ACCEL_HIGH_G_Z_AXIS |   2
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_high_g_axis_enable(
u8 v_channel_u8, u8 *v_data_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel highg enable is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			v_stat_s8 = bno055_write_page_id(PAGE_ONE);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ONE)) {
			switch (v_channel_u8) {
			case BNO055_ACCEL_HIGH_G_X_AXIS:
				/* Read the value of accel x highg enable*/
				com_rslt = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_ACCEL_HIGH_G_X_AXIS__REG,
				&v_data_u8r, BNO055_ONE_U8X);
				*v_data_u8 =
				BNO055_GET_BITSLICE(v_data_u8r,
				BNO055_ACCEL_HIGH_G_X_AXIS);
				break;
			case BNO055_ACCEL_HIGH_G_Y_AXIS:
				/* Read the value of accel y highg enable*/
				com_rslt = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_ACCEL_HIGH_G_Y_AXIS__REG,
				&v_data_u8r, BNO055_ONE_U8X);
				*v_data_u8 =
				BNO055_GET_BITSLICE(v_data_u8r,
				BNO055_ACCEL_HIGH_G_Y_AXIS);
				break;
			case BNO055_ACCEL_HIGH_G_Z_AXIS:
				/* Read the value of accel z highg enable*/
				com_rslt = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_ACCEL_HIGH_G_Z_AXIS__REG,
				&v_data_u8r, BNO055_ONE_U8X);
				*v_data_u8 =
				BNO055_GET_BITSLICE(v_data_u8r,
				BNO055_ACCEL_HIGH_G_Z_AXIS);
				break;
			default:
				com_rslt = E_BNO055_OUT_OF_RANGE;
				break;
			}
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to write the accel highg enable
 *	from page one register from 0x12 bit 5 to 7
 *
 *	@param v_data_u8 : The value of accel highg enable
 *      v_data_u8| result
 *  ------------ | -------------
 *      0x01     | ENABLED
 *      0x00     | DISABLED
 *	@param v_channel_u8 : The value of accel highg axis selection
 *               v_channel_u8     | value
 *     -------------------------- | ----------
 *     BNO055_ACCEL_HIGH_G_X_AXIS |   0
 *     BNO055_ACCEL_HIGH_G_Y_AXIS |   1
 *     BNO055_ACCEL_HIGH_G_Z_AXIS |   2
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_high_g_axis_enable(
u8 v_channel_u8, u8 v_data_u8)
{
BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 v_data_u8r = BNO055_ZERO_U8X;
s8 v_stat_s8 = ERROR;
s8 v_pg_stat_s8 = ERROR;
u8 v_prev_opmode_u8 = OPERATION_MODE_CONFIG;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
} else {
/* The write operation effective only if the operation
mode is in config mode, this part of code is checking the
current operation mode and set the config mode */
v_stat_s8 = bno055_get_operation_mode(&v_prev_opmode_u8);
if (v_stat_s8 == SUCCESS) {
	if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
		v_stat_s8 += bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);
		if (v_stat_s8 == SUCCESS) {
			/* Write page as one */
			v_pg_stat_s8 = bno055_write_page_id(PAGE_ONE);
			if (v_pg_stat_s8 == SUCCESS) {
				switch (v_channel_u8) {
				case BNO055_ACCEL_HIGH_G_X_AXIS:
				/* Write the value of
				accel x highg enable*/
				com_rslt = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_ACCEL_HIGH_G_X_AXIS__REG,
				&v_data_u8r, BNO055_ONE_U8X);
				if (com_rslt == SUCCESS) {
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_ACCEL_HIGH_G_X_AXIS, v_data_u8);
					com_rslt +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_ACCEL_HIGH_G_X_AXIS__REG,
					&v_data_u8r, BNO055_ONE_U8X);
				}
				break;
				case BNO055_ACCEL_HIGH_G_Y_AXIS:
				/* Write the value of
				accel y highg enable*/
				com_rslt = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_ACCEL_HIGH_G_Y_AXIS__REG,
				&v_data_u8r, BNO055_ONE_U8X);
				if (com_rslt == SUCCESS) {
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_ACCEL_HIGH_G_Y_AXIS,
					v_data_u8);
					com_rslt +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_ACCEL_HIGH_G_Y_AXIS__REG,
					&v_data_u8r, BNO055_ONE_U8X);
				}
				break;
				case BNO055_ACCEL_HIGH_G_Z_AXIS:
				/* Write the value of
				accel z highg enable*/
				com_rslt = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_ACCEL_HIGH_G_Z_AXIS__REG,
				&v_data_u8r, BNO055_ONE_U8X);
				if (com_rslt == SUCCESS) {
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_ACCEL_HIGH_G_Z_AXIS, v_data_u8);
					com_rslt +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_ACCEL_HIGH_G_Z_AXIS__REG,
					&v_data_u8r, BNO055_ONE_U8X);
				}
				break;
				default:
				com_rslt = E_BNO055_OUT_OF_RANGE;
				break;
				}
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	} else {
	com_rslt = ERROR;
	}
}
if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
	/* set the operation mode of
	previous operation mode*/
	com_rslt += bno055_set_operation_mode
	(v_prev_opmode_u8);
return com_rslt;
}
/*!
 *	@brief This API used to read the accel highg duration
 *	from page one register from 0x13 bit 0 to 7
 *
 *	@param v_accel_high_g_durn_u8 : The value of accel highg duration
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note The high-g interrupt trigger delay according
 *	to [highg duration  + 1] * 2 ms
 *
 *	in a range from 2 ms to 512 ms
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_high_g_durn(
u8 *v_accel_high_g_durn_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel highg duration is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			v_stat_s8 = bno055_write_page_id(PAGE_ONE);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ONE)) {
			/* Read the value of accel highg duration*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_HIGH_G_DURN__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_accel_high_g_durn_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_ACCEL_HIGH_G_DURN);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to write the accel highg duration
 *	from page one register from 0x13 bit 0 to 7
 *
 *	@param v_accel_high_g_durn_u8 : The value of accel highg duration
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note The high-g interrupt trigger delay according
 *	to [highg duration  + 1] * 2 ms
 *
 *	in a range from 2 ms to 512 ms
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_high_g_durn(
u8 v_accel_high_g_durn_u8)
{
BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 v_data_u8r = BNO055_ZERO_U8X;
s8 v_stat_s8 = ERROR;
u8 v_prev_opmode_u8 = OPERATION_MODE_CONFIG;
s8 v_pg_stat_s8 = ERROR;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
} else {
	/* The write operation effective only if the operation
	mode is in config mode, this part of code is checking the
	current operation mode and set the config mode */
	v_stat_s8 = bno055_get_operation_mode(&v_prev_opmode_u8);
	if (v_stat_s8 == SUCCESS) {
		if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
			v_stat_s8 += bno055_set_operation_mode
			(OPERATION_MODE_CONFIG);

			if (v_stat_s8 == SUCCESS) {
				/* Write page as one */
				v_pg_stat_s8 = bno055_write_page_id(PAGE_ONE);
				if (v_pg_stat_s8 == SUCCESS) {
					com_rslt =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_ACCEL_HIGH_G_DURN__REG,
					&v_data_u8r, BNO055_ONE_U8X);
					if (com_rslt == SUCCESS) {
						/* Write the value of
						accel highg duration*/
						v_data_u8r =
						BNO055_SET_BITSLICE(v_data_u8r,
						BNO055_ACCEL_HIGH_G_DURN,
						v_accel_high_g_durn_u8);
						com_rslt +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_ACCEL_HIGH_G_DURN__REG,
						&v_data_u8r, BNO055_ONE_U8X);
					}
				} else {
				com_rslt = ERROR;
				}
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	}
	if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
		/* set the operation mode of
		previous operation mode*/
		com_rslt += bno055_set_operation_mode
		(v_prev_opmode_u8);
	return com_rslt;
}
/*!
 *	@brief This API used to read the accel highg threshold
 *	from page one register from 0x14 bit 0 to 7
 *
 *	@param v_accel_high_g_thres_u8 : The value of accel highg threshold
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note Accel highg interrupt threshold dependent
 *	for accel g range
 *
 *  v_accel_range_u8	 |	threshold	 |	LSB
 * ------------- | ------------- | ---------
 *     2g        |    7.81mg     |   1LSB
 *     4g        |    15.63mg    |   1LSB
 *     8g        |    31.25mg    |   1LSB
 *     16g       |    62.5mg     |   1LSB
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_high_g_thres(
u8 *v_accel_high_g_thres_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X)  {
		return E_NULL_PTR;
		} else {
		/*condition check for page, highg threshold is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			v_stat_s8 = bno055_write_page_id(PAGE_ONE);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ONE)) {
			/* Read the value of highg threshold */
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_HIGH_G_THRES__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_accel_high_g_thres_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_ACCEL_HIGH_G_THRES);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to write the accel highg threshold
 *	from page one register from 0x14 bit 0 to 7
 *
 *	@param v_accel_high_g_thres_u8 : The value of accel highg threshold
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note Accel highg interrupt threshold dependent
 *	for accel g range
 *
 *  v_accel_range_u8	 |	threshold	 |	LSB
 * ------------- | ------------- | ---------
 *     2g        |    7.81mg     |   1LSB
 *     4g        |    15.63mg    |   1LSB
 *     8g        |    31.25mg    |   1LSB
 *     16g       |    62.5mg     |   1LSB
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_high_g_thres(
u8 v_accel_high_g_thres_u8)
{
BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 v_data_u8r = BNO055_ZERO_U8X;
s8 v_stat_s8 = ERROR;
u8 v_prev_opmode_u8 = OPERATION_MODE_CONFIG;
s8 v_pg_stat_s8 = ERROR;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X)  {
	return E_NULL_PTR;
	} else {
	/* The write operation effective only if the operation
	mode is in config mode, this part of code is checking the
	current operation mode and set the config mode */
	v_stat_s8 = bno055_get_operation_mode(&v_prev_opmode_u8);
	if (v_stat_s8 == SUCCESS) {
		if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
			v_stat_s8 += bno055_set_operation_mode
			(OPERATION_MODE_CONFIG);
			if (v_stat_s8 == SUCCESS) {
				/* Write page as one */
				v_pg_stat_s8 = bno055_write_page_id(PAGE_ONE);
				if (v_pg_stat_s8 == SUCCESS) {
					com_rslt =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_ACCEL_HIGH_G_THRES__REG,
					&v_data_u8r, BNO055_ONE_U8X);
					if (com_rslt == SUCCESS) {
						/* Write the value of
						accel highg threshold */
						v_data_u8r =
						BNO055_SET_BITSLICE(v_data_u8r,
						BNO055_ACCEL_HIGH_G_THRES,
						v_accel_high_g_thres_u8);
						com_rslt +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_ACCEL_HIGH_G_THRES__REG,
						&v_data_u8r, BNO055_ONE_U8X);
					}
				} else {
				com_rslt = ERROR;
				}
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	}
	if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
		/* set the operation mode of
		previous operation mode*/
		com_rslt += bno055_set_operation_mode
		(v_prev_opmode_u8);
	return com_rslt;
}
/*!
 *	@brief This API used to read the accel slownomotion threshold
 *	from page one register from 0x15 bit 0 to 7
 *
 *	@param v_accel_slow_no_motion_thres_u8 :
 *	The value of accel slownomotion threshold
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note Accel slow no motion interrupt threshold dependent
 *	for accel g range
 *
 *  v_accel_range_u8	 |	threshold	 |	LSB
 * ------------- | ------------- | ---------
 *     2g        |    3.19mg     |   1LSB
 *     4g        |    7.81mg     |   1LSB
 *     8g        |    15.63mg    |   1LSB
 *     16g       |    31.25mg    |   1LSB
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_slow_no_motion_thres(
u8 *v_accel_slow_no_motion_thres_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X)  {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel slownomotion threshold is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			v_stat_s8 = bno055_write_page_id(PAGE_ONE);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ONE)) {
			/* Read the value of slownomotion threshold */
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_SLOW_NO_MOTION_THRES__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_accel_slow_no_motion_thres_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_ACCEL_SLOW_NO_MOTION_THRES);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to write the accel slownomotion threshold
 *	from page one register from 0x15 bit 0 to 7
 *
 *	@param v_accel_slow_no_motion_thres_u8 :
 *	The value of accel slownomotion threshold
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note Accel slow no motion interrupt threshold dependent
 *	for accel g range
 *
 *  v_accel_range_u8	 |	threshold	 |	LSB
 * ------------- | ------------- | ---------
 *     2g        |    3.19mg     |   1LSB
 *     4g        |    7.81mg     |   1LSB
 *     8g        |    15.63mg    |   1LSB
 *     16g       |    31.25mg    |   1LSB
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_slow_no_motion_thres(
u8 v_accel_slow_no_motion_thres_u8)
{
BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 v_data_u8r = BNO055_ZERO_U8X;
s8 v_stat_s8 = ERROR;
u8 v_prev_opmode_u8 = OPERATION_MODE_CONFIG;
s8 v_pg_stat_s8 = ERROR;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X)  {
	return E_NULL_PTR;
} else {
/* The write operation effective only if the operation
mode is in config mode, this part of code is checking the
current operation mode and set the config mode */
v_stat_s8 = bno055_get_operation_mode(&v_prev_opmode_u8);
if (v_stat_s8 == SUCCESS) {
	if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
		v_stat_s8 += bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);
		if (v_stat_s8 == SUCCESS) {
			/* Write page as one */
			v_pg_stat_s8 = bno055_write_page_id(PAGE_ONE);
			if (v_pg_stat_s8 == SUCCESS) {
				/* Write the value of
				slownomotion threshold */
				com_rslt = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_ACCEL_SLOW_NO_MOTION_THRES__REG,
				&v_data_u8r, BNO055_ONE_U8X);
				if (com_rslt == SUCCESS) {
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_ACCEL_SLOW_NO_MOTION_THRES,
					v_accel_slow_no_motion_thres_u8);
					com_rslt +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_ACCEL_SLOW_NO_MOTION_THRES__REG,
					&v_data_u8r, BNO055_ONE_U8X);
				}
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	} else {
	com_rslt = ERROR;
	}
}
if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
	/* set the operation mode of
	previous operation mode*/
	com_rslt += bno055_set_operation_mode
	(v_prev_opmode_u8);
return com_rslt;
}
/*!
 *	@brief This API used to read accel slownomotion enable
 *	from page one register from 0x16 bit 0
 *
 *	@param v_accel_slow_no_motion_en_u8 : The value of accel slownomotion enable
 *	  v_accel_slow_no_motion_en_u8   | result
 *     ------------------------      | --------
 *              0x01                 | Slow motion
 *              0x00                 | No motion
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_slow_no_motion_enable(
u8 *v_accel_slow_no_motion_en_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X)  {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel slownomotion enable is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			v_stat_s8 = bno055_write_page_id(PAGE_ONE);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ONE)) {
			/* Read the value of accel slownomotion enable */
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_SLOW_NO_MOTION_ENABLE__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_accel_slow_no_motion_en_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_ACCEL_SLOW_NO_MOTION_ENABLE);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to write accel slownomotion enable
 *	from page one register from 0x16 bit 0
 *
 *	@param v_accel_slow_no_motion_en_u8 : The value of accel slownomotion enable
 *	  v_accel_slow_no_motion_en_u8   | result
 *     ------------------------      | --------
 *              0x01                 | Slow motion
 *              0x00                 | No motion
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_slow_no_motion_enable(
u8 v_accel_slow_no_motion_en_u8)
{
BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 v_data_u8r = BNO055_ZERO_U8X;
s8 v_stat_s8 = ERROR;
u8 v_prev_opmode_u8 = OPERATION_MODE_CONFIG;
s8 v_pg_stat_s8 = ERROR;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X)  {
	return E_NULL_PTR;
} else {
/* The write operation effective only if the operation
mode is in config mode, this part of code is checking the
current operation mode and set the config mode */
v_stat_s8 = bno055_get_operation_mode(&v_prev_opmode_u8);
if (v_stat_s8 == SUCCESS) {
	if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
		v_stat_s8 += bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);
		if (v_stat_s8 == SUCCESS) {
			/* Write page as one */
			v_pg_stat_s8 = bno055_write_page_id(PAGE_ONE);
			if (v_pg_stat_s8 == SUCCESS) {
				com_rslt = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_ACCEL_SLOW_NO_MOTION_ENABLE__REG,
				&v_data_u8r, BNO055_ONE_U8X);
				if (com_rslt == SUCCESS) {
					/* Read the value of
					accel slownomotion enable */
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_ACCEL_SLOW_NO_MOTION_ENABLE,
					v_accel_slow_no_motion_en_u8);
					com_rslt +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_ACCEL_SLOW_NO_MOTION_ENABLE__REG,
					&v_data_u8r, BNO055_ONE_U8X);
				}
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	} else {
	com_rslt = ERROR;
	}
}
if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
	/* set the operation mode of
	previous operation mode*/
	com_rslt += bno055_set_operation_mode
	(v_prev_opmode_u8);
return com_rslt;
}
/*!
 *	@brief This API used to read accel slownomotion duration
 *	from page one register from 0x16 bit 1 to 6
 *
 *	@param v_accel_slow_no_motion_durn_u8 :
 *	The value of accel slownomotion duration
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_slow_no_motion_durn(
u8 *v_accel_slow_no_motion_durn_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X)  {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel slownomotion duration is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			v_stat_s8 = bno055_write_page_id(PAGE_ONE);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ONE)) {
			/*read value of accel slownomotion duration*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_SLOW_NO_MOTION_DURN__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_accel_slow_no_motion_durn_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_ACCEL_SLOW_NO_MOTION_DURN);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to write accel slownomotion duration
 *	from page one register from 0x16 bit 1 to 6
 *
 *	@param v_accel_slow_no_motion_durn_u8 :
 *	The value of accel slownomotion duration
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_slow_no_motion_durn(
u8 v_accel_slow_no_motion_durn_u8)
{
BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 v_data_u8r = BNO055_ZERO_U8X;
s8 v_stat_s8 = ERROR;
u8 v_prev_opmode_u8 = OPERATION_MODE_CONFIG;
s8 v_pg_stat_s8 = ERROR;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X)  {
	return E_NULL_PTR;
} else {
/* The write operation effective only if the operation
mode is in config mode, this part of code is checking the
current operation mode and set the config mode */
v_stat_s8 = bno055_get_operation_mode(&v_prev_opmode_u8);
if (v_stat_s8 == SUCCESS) {
	if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
		v_stat_s8 += bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);
		if (v_stat_s8 == SUCCESS) {
			/* Write page as one */
			v_pg_stat_s8 = bno055_write_page_id(PAGE_ONE);
			if (v_pg_stat_s8 == SUCCESS) {
				com_rslt = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_ACCEL_SLOW_NO_MOTION_DURN__REG,
				&v_data_u8r, BNO055_ONE_U8X);
				if (com_rslt == SUCCESS) {
					/*Write the value of accel
					slownomotion duration*/
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_ACCEL_SLOW_NO_MOTION_DURN,
					v_accel_slow_no_motion_durn_u8);
					com_rslt +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_ACCEL_SLOW_NO_MOTION_DURN__REG,
					&v_data_u8r, BNO055_ONE_U8X);
				}
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	} else {
	com_rslt = ERROR;
	}
}
if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
	/* set the operation mode of
	previous operation mode*/
	com_rslt += bno055_set_operation_mode
	(v_prev_opmode_u8);
return com_rslt;
}
/*!
 *	@brief This API used to read the gyro anymotion enable
 *	from page one register from 0x17 bit 0 to 2
 *
 *	@param v_data_u8 : The value of gyro anymotion enable
 *      v_data_u8     | result
 *  ----------------- |-------------
 *      0x01          | ENABLED
 *      0x00          | DISABLED
 *	@param v_channel_u8 : The value of gyro anymotion axis selection
 *               v_channel_u8         | value
 *     ---------------------------    | ----------
 *     BNO055_GYRO_ANY_MOTIONX_AXIS   |   0
 *     BNO055_GYRO_ANY_MOTIONY_AXIS   |   1
 *     BNO055_GYRO_ANY_MOTIONZ_AXIS   |   2
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_any_motion_axis_enable(
u8 v_channel_u8, u8 *v_data_u8)
{
/* Variable used to return value of
communication routine*/
BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 v_data_u8r = BNO055_ZERO_U8X;
s8 v_stat_s8 = ERROR;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X)  {
	return E_NULL_PTR;
	} else {
	/*condition check for page, gyro anymotion axis is
	available in the page one*/
	if (p_bno055->page_id != PAGE_ONE)
		/* Write page as one */
		v_stat_s8 = bno055_write_page_id(PAGE_ONE);
	if ((v_stat_s8 == SUCCESS) ||
	(p_bno055->page_id == PAGE_ONE)) {
		switch (v_channel_u8) {
		case BNO055_GYRO_ANY_MOTION_X_AXIS:
			/* Read the gyro anymotion x enable*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_ANY_MOTION_X_AXIS__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_data_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_GYRO_ANY_MOTION_X_AXIS);
			break;
		case BNO055_GYRO_ANY_MOTION_Y_AXIS:
			/* Read the gyro anymotion y enable*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_ANY_MOTION_Y_AXIS__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_data_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_GYRO_ANY_MOTION_Y_AXIS);
			break;
		case BNO055_GYRO_ANY_MOTION_Z_AXIS:
			/* Read the gyro anymotion z enable*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_ANY_MOTION_Z_AXIS__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_data_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_GYRO_ANY_MOTION_Z_AXIS);
			break;
		default:
			com_rslt = E_BNO055_OUT_OF_RANGE;
			break;
		}
	} else {
	com_rslt = ERROR;
	}
}
return com_rslt;
}
/*!
 *	@brief This API used to write the gyro anymotion enable
 *	from page one register from 0x17 bit 0 to 2
 *
 *	@param v_data_u8 : The value of gyro anymotion enable
 *      v_data_u8     | result
 *  ----------------- |-------------
 *      0x01          | ENABLED
 *      0x00          | DISABLED
 *	@param v_channel_u8 : The value of gyro anymotion axis selection
 *               v_channel_u8         | value
 *     ---------------------------    | ----------
 *     BNO055_GYRO_ANY_MOTIONX_AXIS   |   0
 *     BNO055_GYRO_ANY_MOTIONY_AXIS   |   1
 *     BNO055_GYRO_ANY_MOTIONZ_AXIS   |   2
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_any_motion_axis_enable(
u8 v_channel_u8, u8  v_data_u8)
{
BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 v_data_u8r = BNO055_ZERO_U8X;
s8 v_stat_s8 = ERROR;
u8 v_prev_opmode_u8 = OPERATION_MODE_CONFIG;
s8 v_pg_stat_s8 = ERROR;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X)  {
	return E_NULL_PTR;
} else {
/* The write operation effective only if the operation
mode is in config mode, this part of code is checking the
current operation mode and set the config mode */
v_stat_s8 = bno055_get_operation_mode(&v_prev_opmode_u8);
if (v_stat_s8 == SUCCESS) {
	if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
		v_stat_s8 += bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);
	if (v_stat_s8 == SUCCESS) {
		/* Write page as one */
		v_pg_stat_s8 = bno055_write_page_id(PAGE_ONE);
		if (v_pg_stat_s8 == SUCCESS) {
			switch (v_channel_u8) {
			case BNO055_GYRO_ANY_MOTION_X_AXIS:
				/* Write the gyro
				anymotion x enable*/
				com_rslt =
				p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_GYRO_ANY_MOTION_X_AXIS__REG,
				&v_data_u8r, BNO055_ONE_U8X);
			if (com_rslt == SUCCESS) {
				v_data_u8r = BNO055_SET_BITSLICE
				(v_data_u8r,
				BNO055_GYRO_ANY_MOTION_X_AXIS,
				v_data_u8);
				com_rslt +=
				p_bno055->BNO055_BUS_WRITE_FUNC
				(p_bno055->dev_addr,
				BNO055_GYRO_ANY_MOTION_X_AXIS__REG,
				&v_data_u8r, BNO055_ONE_U8X);
			}
			break;
			case BNO055_GYRO_ANY_MOTION_Y_AXIS:
				/* Write the gyro
				anymotion y enable*/
				com_rslt =
				p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_GYRO_ANY_MOTION_Y_AXIS__REG,
				&v_data_u8r, BNO055_ONE_U8X);
			if (com_rslt == SUCCESS) {
				v_data_u8r = BNO055_SET_BITSLICE
				(v_data_u8r,
				BNO055_GYRO_ANY_MOTION_Y_AXIS, v_data_u8);
				com_rslt +=
				p_bno055->BNO055_BUS_WRITE_FUNC
				(p_bno055->dev_addr,
				BNO055_GYRO_ANY_MOTION_Y_AXIS__REG,
				&v_data_u8r, BNO055_ONE_U8X);
			}
			break;
			case BNO055_GYRO_ANY_MOTION_Z_AXIS:
				/* Write the gyro
				anymotion z enable*/
				com_rslt =
				p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_GYRO_ANY_MOTION_Z_AXIS__REG,
				&v_data_u8r, BNO055_ONE_U8X);
			if (com_rslt == SUCCESS) {
				v_data_u8r = BNO055_SET_BITSLICE
				(v_data_u8r,
				BNO055_GYRO_ANY_MOTION_Z_AXIS,
				v_data_u8);
				com_rslt +=
				p_bno055->BNO055_BUS_WRITE_FUNC
				(p_bno055->dev_addr,
				BNO055_GYRO_ANY_MOTION_Z_AXIS__REG,
				&v_data_u8r, BNO055_ONE_U8X);
			}
			break;
			default:
				com_rslt = E_BNO055_OUT_OF_RANGE;
				break;
			}
		} else {
		com_rslt = ERROR;
		}
	} else {
	com_rslt = ERROR;
	}
} else {
com_rslt = ERROR;
}
}
if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
	/* set the operation mode of
	previous operation mode*/
	com_rslt += bno055_set_operation_mode
	(v_prev_opmode_u8);
return com_rslt;
}
/*!
 *	@brief This API used to read the gyro highrate enable
 *	from page one register from 0x17 bit 3 to 5
 *
 *	@param v_data_u8 : The value of gyro highrate enable
 *      v_data_u8     | result
 *  ----------------  |-------------
 *      0x01          | ENABLED
 *      0x00          | DISABLED
 *	@param v_channel_u8 : The value of gyro highrate axis selection
 *               v_channel_u8         | value
 *     ------------------------       | ----------
 *     BNO055_GYRO_HIGHRATE_X_AXIS    |   0
 *     BNO055_GYRO_HIGHRATE_Y_AXIS    |   1
 *     BNO055_GYRO_HIGHRATE_Z_AXIS    |   2
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_axis_enable(
u8 v_channel_u8, u8 *v_data_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X)  {
		return E_NULL_PTR;
		} else {
		/*condition check for page, gyro highrate enable is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			v_stat_s8 = bno055_write_page_id(PAGE_ONE);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ONE)) {
			switch (v_channel_u8) {
			case BNO055_GYRO_HIGHRATE_X_AXIS:
				/* Read the gyro highrate x enable */
				com_rslt = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_GYRO_HIGHRATE_X_AXIS__REG,
				&v_data_u8r, BNO055_ONE_U8X);
				*v_data_u8 =
				BNO055_GET_BITSLICE(v_data_u8r,
				BNO055_GYRO_HIGHRATE_X_AXIS);
				break;
			case BNO055_GYRO_HIGHRATE_Y_AXIS:
				/* Read the gyro highrate y enable */
				com_rslt = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_GYRO_HIGHRATE_Y_AXIS__REG,
				&v_data_u8r, BNO055_ONE_U8X);
				*v_data_u8 =
				BNO055_GET_BITSLICE(v_data_u8r,
				BNO055_GYRO_HIGHRATE_Y_AXIS);
				break;
			case BNO055_GYRO_HIGHRATE_Z_AXIS:
				/* Read the gyro highrate z enable */
				com_rslt = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_GYRO_HIGHRATE_Z_AXIS__REG,
				&v_data_u8r, BNO055_ONE_U8X);
				*v_data_u8 =
				BNO055_GET_BITSLICE(v_data_u8r,
				BNO055_GYRO_HIGHRATE_Z_AXIS);
				break;
			default:
				com_rslt = E_BNO055_OUT_OF_RANGE;
				break;
			}
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to write the gyro highrate enable
 *	from page one register from 0x17 bit 3 to 5
 *
 *	@param v_data_u8 : The value of gyro highrate enable
 *      v_data_u8     | result
 *  ----------------  |-------------
 *      0x01          | ENABLED
 *      0x00          | DISABLED
 *	@param v_channel_u8 : The value of gyro highrate axis selection
 *               v_channel_u8         | value
 *     ------------------------       | ----------
 *     BNO055_GYRO_HIGHRATE_X_AXIS    |   0
 *     BNO055_GYRO_HIGHRATE_Y_AXIS    |   1
 *     BNO055_GYRO_HIGHRATE_Z_AXIS    |   2
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_axis_enable(
u8 v_channel_u8, u8 v_data_u8)
{
BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 v_data_u8r = BNO055_ZERO_U8X;
s8 v_stat_s8 = ERROR;
u8 v_prev_opmode_u8 = OPERATION_MODE_CONFIG;
s8 v_pg_stat_s8 = ERROR;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X)  {
	return E_NULL_PTR;
} else {
/* The write operation effective only if the operation
mode is in config mode, this part of code is checking the
current operation mode and set the config mode */
v_stat_s8 = bno055_get_operation_mode(&v_prev_opmode_u8);
if (v_stat_s8 == SUCCESS) {
	if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
		v_stat_s8 += bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);
		if (v_stat_s8 == SUCCESS) {
			/* Write page as one */
			v_pg_stat_s8 = bno055_write_page_id(PAGE_ONE);
			if (v_pg_stat_s8 == SUCCESS) {
				switch (v_channel_u8) {
				case BNO055_GYRO_HIGHRATE_X_AXIS:
				/* Write the value of
				gyro highrate x enable*/
				com_rslt = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_GYRO_HIGHRATE_X_AXIS__REG,
				&v_data_u8r, BNO055_ONE_U8X);
				if (com_rslt == SUCCESS) {
					v_data_u8r = BNO055_SET_BITSLICE
					(v_data_u8r,
					BNO055_GYRO_HIGHRATE_X_AXIS, v_data_u8);
					com_rslt +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_GYRO_HIGHRATE_X_AXIS__REG,
					&v_data_u8r, BNO055_ONE_U8X);
				}
				break;
				case BNO055_GYRO_HIGHRATE_Y_AXIS:
				/* Write the value of
				gyro highrate y enable*/
				com_rslt = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_GYRO_HIGHRATE_Y_AXIS__REG,
				&v_data_u8r, BNO055_ONE_U8X);
				if (com_rslt == SUCCESS) {
					v_data_u8r =
					BNO055_SET_BITSLICE(
					v_data_u8r,
					BNO055_GYRO_HIGHRATE_Y_AXIS, v_data_u8);
					com_rslt +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_GYRO_HIGHRATE_Y_AXIS__REG,
					&v_data_u8r, BNO055_ONE_U8X);
				}
				break;
				case BNO055_GYRO_HIGHRATE_Z_AXIS:
				/* Write the value of
				gyro highrate z enable*/
				com_rslt =
				p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_GYRO_HIGHRATE_Z_AXIS__REG,
				&v_data_u8r, BNO055_ONE_U8X);
				if (com_rslt == SUCCESS) {
					v_data_u8r = BNO055_SET_BITSLICE
					(v_data_u8r,
					BNO055_GYRO_HIGHRATE_Z_AXIS, v_data_u8);
					com_rslt +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_GYRO_HIGHRATE_Z_AXIS__REG,
					&v_data_u8r, BNO055_ONE_U8X);
				}
				break;
				default:
				com_rslt = E_BNO055_OUT_OF_RANGE;
				break;
				}
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	} else {
	com_rslt = ERROR;
	}
}
if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
	/* set the operation mode of
	previous operation mode*/
	com_rslt += bno055_set_operation_mode
	(v_prev_opmode_u8);
return com_rslt;
}
/*!
 *	@brief This API used to read gyro anymotion filter
 *	from page one register from 0x17 bit 6
 *
 *	@param v_gyro_any_motion_filter_u8 : The value of gyro anymotion filter
 *   v_gyro_any_motion_filter_u8  | result
 *  ---------------------------   |------------
 *      0x00                      | FILTERED
 *      0x01                      | UNFILTERED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_any_motion_filter(
u8 *v_gyro_any_motion_filter_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X)  {
		return E_NULL_PTR;
		} else {
		/*condition check for page, gyro anymotion filter is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			v_stat_s8 = bno055_write_page_id(PAGE_ONE);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ONE)) {
			/* Read the value of gyro anymotion filter*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_ANY_MOTION_FILTER__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_gyro_any_motion_filter_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_GYRO_ANY_MOTION_FILTER);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to write gyro anymotion filter
 *	from page one register from 0x17 bit 6
 *
 *	@param v_gyro_any_motion_filter_u8 : The value of gyro anymotion filter
 *   v_gyro_any_motion_filter_u8  | result
 *  ---------------------------   |------------
 *      0x00                      | FILTERED
 *      0x01                      | UNFILTERED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_any_motion_filter(
u8 v_gyro_any_motion_filter_u8)
{
BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 v_data_u8r = BNO055_ZERO_U8X;
s8 v_stat_s8 = ERROR;
u8 v_prev_opmode_u8 = OPERATION_MODE_CONFIG;
s8 v_pg_stat_s8 = ERROR;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X)  {
	return E_NULL_PTR;
} else {
/* The write operation effective only if the operation
mode is in config mode, this part of code is checking the
current operation mode and set the config mode */
v_stat_s8 = bno055_get_operation_mode(&v_prev_opmode_u8);
if (v_stat_s8 == SUCCESS) {
	if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
		v_stat_s8 += bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);
		if (v_stat_s8 == SUCCESS) {
			/* Write page as one */
			v_pg_stat_s8 = bno055_write_page_id(PAGE_ONE);
			if (v_pg_stat_s8 == SUCCESS) {
				/* Write the value of
				gyro anymotion filter*/
				com_rslt = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_GYRO_ANY_MOTION_FILTER__REG,
				&v_data_u8r, BNO055_ONE_U8X);
				if (com_rslt == SUCCESS) {
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_GYRO_ANY_MOTION_FILTER,
					v_gyro_any_motion_filter_u8);
					com_rslt +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_GYRO_ANY_MOTION_FILTER__REG,
					&v_data_u8r, BNO055_ONE_U8X);
				}
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	} else {
	com_rslt = ERROR;
	}
}
if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
	/* set the operation mode of
	previous operation mode*/
	com_rslt += bno055_set_operation_mode
	(v_prev_opmode_u8);
return com_rslt;
}
/*!
 *	@brief This API used to read gyro highrate filter
 *	from page one register from 0x17 bit 7
 *
 *	@param v_gyro_highrate_filter_u8 : The value of gyro highrate filter
 *   v_gyro_highrate_filter_u8  | result
 *  --------------------------- |------------
 *         0x00                 | FILTERED
 *         0x01                 | UNFILTERED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_filter(
u8 *v_gyro_highrate_filter_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X)  {
		return E_NULL_PTR;
		} else {
		/*condition check for page, gyro highrate filter is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			v_stat_s8 = bno055_write_page_id(PAGE_ONE);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ONE)) {
			/* Read the value of gyro highrate filter */
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_HIGHRATE_FILTER__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_gyro_highrate_filter_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_GYRO_HIGHRATE_FILTER);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to write gyro highrate filter
 *	from page one register from 0x17 bit 7
 *
 *	@param v_gyro_highrate_filter_u8 : The value of gyro highrate filter
 *   v_gyro_highrate_filter_u8  | result
 *  --------------------------- |------------
 *         0x00                 | FILTERED
 *         0x01                 | UNFILTERED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_filter(
u8 v_gyro_highrate_filter_u8)
{
BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 v_data_u8r = BNO055_ZERO_U8X;
s8 v_stat_s8 = ERROR;
s8 v_pg_stat_s8 = ERROR;
u8 v_prev_opmode_u8 = OPERATION_MODE_CONFIG;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X)  {
	return E_NULL_PTR;
} else {
/* The write operation effective only if the operation
mode is in config mode, this part of code is checking the
current operation mode and set the config mode */
v_stat_s8 = bno055_get_operation_mode(&v_prev_opmode_u8);
if (v_stat_s8 == SUCCESS) {
	if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
		v_stat_s8 += bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);
		if (v_stat_s8 == SUCCESS) {
			/* Write page as one */
			v_pg_stat_s8 = bno055_write_page_id(PAGE_ONE);
			if (v_pg_stat_s8 == SUCCESS) {
				/* Write the value of
				gyro highrate filter*/
				com_rslt =
				p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_GYRO_HIGHRATE_FILTER__REG,
				&v_data_u8r,
				BNO055_ONE_U8X);
				if (com_rslt == SUCCESS) {
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_GYRO_HIGHRATE_FILTER,
					v_gyro_highrate_filter_u8);
					com_rslt +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_GYRO_HIGHRATE_FILTER__REG,
					&v_data_u8r, BNO055_ONE_U8X);
				}
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	} else {
	com_rslt = ERROR;
	}
}
if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
	/* set the operation mode of
	previous operation mode*/
	com_rslt += bno055_set_operation_mode
	(v_prev_opmode_u8);
return com_rslt;
}
/*!
 *	@brief This API used to read gyro highrate x threshold
 *	from page one register from 0x18 bit 0 to 4
 *
 *	@param v_gyro_highrate_x_thres_u8 : The value of gyro x highrate threshold
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note Gyro highrate threshold dependent on the
 *	selection of gyro range
 *
 *  v_gyro_range_u8	  |	threshold		|     LSB
 * -----------------  | -------------   | ---------
 *     2000           |    62.5dps      |   1LSB
 *     1000           |    31.25dps     |   1LSB
 *     500            |    15.625dps    |   1LSB
 *     125            |    7.8125dps    |   1LSB
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_x_thres(
u8 *v_gyro_highrate_x_thres_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X)  {
		return E_NULL_PTR;
		} else {
		/*condition check for page, gyro highrate x threshold is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			v_stat_s8 = bno055_write_page_id(PAGE_ONE);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ONE)) {
			/* Read the value of gyro highrate threshold*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_HIGHRATE_X_THRES__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_gyro_highrate_x_thres_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_GYRO_HIGHRATE_X_THRES);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to write gyro highrate x threshold
 *	from page one register from 0x18 bit 0 to 4
 *
 *	@param v_gyro_highrate_x_thres_u8 : The value of gyro x highrate threshold
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note Gyro highrate threshold dependent on the
 *	selection of gyro range
 *
 *  v_gyro_range_u8	  |	threshold		|     LSB
 * -----------------  | -------------   | ---------
 *     2000           |    62.5dps      |   1LSB
 *     1000           |    31.25dps     |   1LSB
 *     500            |    15.625dps    |   1LSB
 *     125            |    7.8125dps    |   1LSB
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_x_thres(
u8 v_gyro_highrate_x_thres_u8)
{
BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 v_data_u8r = BNO055_ZERO_U8X;
s8 v_stat_s8 = ERROR;
u8 v_prev_opmode_u8 = OPERATION_MODE_CONFIG;
s8 v_pg_stat_s8 = ERROR;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X)  {
	return E_NULL_PTR;
} else {
/* The write operation effective only if the operation
mode is in config mode, this part of code is checking the
current operation mode and set the config mode */
v_stat_s8 = bno055_get_operation_mode(&v_prev_opmode_u8);
if (v_stat_s8 == SUCCESS) {
	if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
		v_stat_s8 += bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);
		if (v_stat_s8 == SUCCESS) {
			/* Write page as one */
			v_pg_stat_s8 = bno055_write_page_id(PAGE_ONE);
			if (v_pg_stat_s8 == SUCCESS) {
				/* Write the value of
				gyro highrate x threshold*/
				com_rslt = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_GYRO_HIGHRATE_X_THRES__REG,
				&v_data_u8r, BNO055_ONE_U8X);
				if (com_rslt == SUCCESS) {
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_GYRO_HIGHRATE_X_THRES,
					v_gyro_highrate_x_thres_u8);
					com_rslt +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_GYRO_HIGHRATE_X_THRES__REG,
					&v_data_u8r, BNO055_ONE_U8X);
				}
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	} else {
	com_rslt = ERROR;
	}
}
if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
	/* set the operation mode of
	previous operation mode*/
	com_rslt += bno055_set_operation_mode
	(v_prev_opmode_u8);
return com_rslt;
}
/*!
 *	@brief This API used to read gyro highrate x hysteresis
 *	from page one register from 0x18 bit 5 to 6
 *
 *	@param v_gyro_highrate_x_hyst_u8 : The value of gyro highrate x hysteresis
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note Gyro high rate hysteresis calculated by
 *
 *	using this (255 + 256 * v_gyro_highrate_x_hyst_u8) *4 LSB
 *
 *	The high rate value scales with the range setting
 *
 *  v_gyro_range_u8	  |	hysteresis		|     LSB
 * -----------------  | -------------   | ---------
 *     2000           |    62.26dps     |   1LSB
 *     1000           |    31.13dps     |   1LSB
 *     500            |    15.56dps     |   1LSB
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_x_hyst(
u8 *v_gyro_highrate_x_hyst_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X)  {
		return E_NULL_PTR;
		} else {
		/*condition check for page,gyro highrate x hysteresis is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			v_stat_s8 = bno055_write_page_id(PAGE_ONE);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ONE)) {
			/* Read the value of gyro highrate x hysteresis*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_HIGHRATE_X_HYST__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_gyro_highrate_x_hyst_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_GYRO_HIGHRATE_X_HYST);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to write gyro highrate x hysteresis
 *	from page one register from 0x18 bit 5 to 6
 *
 *	@param v_gyro_highrate_x_hyst_u8 : The value of gyro highrate x hysteresis
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note Gyro high rate hysteresis calculated by
 *
 *	using this (255 + 256 * v_gyro_highrate_x_hyst_u8) *4 LSB
 *
 *	The high rate value scales with the range setting
 *
 *  v_gyro_range_u8	  |	hysteresis		|     LSB
 * -----------------  | -------------   | ---------
 *     2000           |    62.26dps     |   1LSB
 *     1000           |    31.13dps     |   1LSB
 *     500            |    15.56dps     |   1LSB
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_x_hyst(
u8 v_gyro_highrate_x_hyst_u8)
{
BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 v_data_u8r = BNO055_ZERO_U8X;
s8 v_stat_s8 = ERROR;
u8 v_prev_opmode_u8 = OPERATION_MODE_CONFIG;
s8 v_pg_stat_s8 = ERROR;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X)  {
	return E_NULL_PTR;
} else {
/* The write operation effective only if the operation
mode is in config mode, this part of code is checking the
current operation mode and set the config mode */
v_stat_s8 = bno055_get_operation_mode(&v_prev_opmode_u8);
if (v_stat_s8 == SUCCESS) {
	if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
		v_stat_s8 += bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);
		if (v_stat_s8 == SUCCESS) {
			/* Write page as one */
			v_pg_stat_s8 = bno055_write_page_id(PAGE_ONE);
			if (v_pg_stat_s8 == SUCCESS) {
				/*Write the value of
				gyro highrate x hysteresis*/
				com_rslt = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_GYRO_HIGHRATE_X_HYST__REG,
				&v_data_u8r, BNO055_ONE_U8X);
				if (com_rslt == SUCCESS) {
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_GYRO_HIGHRATE_X_HYST,
					v_gyro_highrate_x_hyst_u8);
					com_rslt +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_GYRO_HIGHRATE_X_HYST__REG,
					&v_data_u8r, BNO055_ONE_U8X);
				}
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	} else {
	com_rslt = ERROR;
	}
}
if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
	/* set the operation mode of
	previous operation mode*/
	com_rslt += bno055_set_operation_mode
	(v_prev_opmode_u8);
return com_rslt;
}
/*!
 *	@brief This API used to read gyro highrate x duration
 *	from page one register from 0x19 bit 0 to 7
 *
 *	@param v_gyro_highrate_x_durn_u8 : The value of gyro highrate x duration
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note Gyro highrate duration calculate by using the formula
 *
 *	(1 + v_gyro_highrate_x_durn_u8)*2.5ms
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_x_durn(
u8 *v_gyro_highrate_x_durn_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X)  {
		return E_NULL_PTR;
		} else {
		/*condition check for page, gyro highrate x duration is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			v_stat_s8 = bno055_write_page_id(PAGE_ONE);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ONE)) {
			/* Read the value of gyro highrate x duration*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_HIGHRATE_X_DURN__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_gyro_highrate_x_durn_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_GYRO_HIGHRATE_X_DURN);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to write gyro highrate x duration
 *	from page one register from 0x19 bit 0 to 7
 *
 *	@param v_gyro_highrate_x_durn_u8 : The value of gyro highrate x duration
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note Gyro highrate duration calculate by using the formula
 *
 *	(1 + v_gyro_highrate_x_durn_u8)*2.5ms
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_x_durn(
u8 v_gyro_highrate_x_durn_u8)
{
BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 v_data_u8r = BNO055_ZERO_U8X;
s8 v_stat_s8 = ERROR;
u8 v_prev_opmode_u8 = OPERATION_MODE_CONFIG;
s8 v_pg_stat_s8 = ERROR;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X)  {
	return E_NULL_PTR;
	} else {
/* The write operation effective only if the operation
mode is in config mode, this part of code is checking the
current operation mode and set the config mode */
v_stat_s8 = bno055_get_operation_mode(&v_prev_opmode_u8);
if (v_stat_s8 == SUCCESS) {
	if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
		v_stat_s8 += bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);
		if (v_stat_s8 == SUCCESS) {
			/* Write page as one */
			v_pg_stat_s8 = bno055_write_page_id(PAGE_ONE);
			if (v_pg_stat_s8 == SUCCESS) {
				/* Write the value
				of gyro highrate x duration*/
				com_rslt = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_GYRO_HIGHRATE_X_DURN__REG,
				&v_data_u8r, BNO055_ONE_U8X);
				if (com_rslt == SUCCESS) {
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_GYRO_HIGHRATE_X_DURN,
					v_gyro_highrate_x_durn_u8);
					com_rslt +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_GYRO_HIGHRATE_X_DURN__REG,
					&v_data_u8r, BNO055_ONE_U8X);
				}
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	} else {
	com_rslt = ERROR;
	}
	}
	if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
		/* set the operation mode of
		previous operation mode*/
		com_rslt += bno055_set_operation_mode
		(v_prev_opmode_u8);
	return com_rslt;
}
/*!
 *	@brief This API used to read gyro highrate y threshold
 *	from page one register from 0x1A bit 0 to 4
 *
 *	@param v_gyro_highrate_y_thres_u8 : The value of gyro highrate y threshold
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note Gyro highrate threshold dependent on the
 *	selection of gyro range
 *
 *  v_gyro_range_u8	  |	threshold		|     LSB
 * -----------------  | -------------   | ---------
 *     2000           |    62.5dps      |   1LSB
 *     1000           |    31.25dps     |   1LSB
 *     500            |    15.625dps    |   1LSB
 *     125            |    7.8125dps    |   1LSB
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_y_thres(
u8 *v_gyro_highrate_y_thres_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X)  {
		return E_NULL_PTR;
		} else {
		/*condition check for page, gyro highrate y threshold is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			v_stat_s8 = bno055_write_page_id(PAGE_ONE);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ONE)) {
			/* Read the value of gyro highrate y threshold*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_HIGHRATE_Y_THRES__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_gyro_highrate_y_thres_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_GYRO_HIGHRATE_Y_THRES);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to write gyro highrate y threshold
 *	from page one register from 0x1A bit 0 to 4
 *
 *	@param v_gyro_highrate_y_thres_u8 : The value of gyro highrate y threshold
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note Gyro highrate threshold dependent on the
 *	selection of gyro range
 *
 *  v_gyro_range_u8	  |	threshold		|     LSB
 * -----------------  | -------------   | ---------
 *     2000           |    62.5dps      |   1LSB
 *     1000           |    31.25dps     |   1LSB
 *     500            |    15.625dps    |   1LSB
 *     125            |    7.8125dps    |   1LSB
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_y_thres(
u8 v_gyro_highrate_y_thres_u8)
{
BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 v_data_u8r = BNO055_ZERO_U8X;
s8 v_stat_s8 = ERROR;
u8 v_prev_opmode_u8 = OPERATION_MODE_CONFIG;
s8 v_pg_stat_s8 = ERROR;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X)  {
	return E_NULL_PTR;
} else {
/* The write operation effective only if the operation
mode is in config mode, this part of code is checking the
current operation mode and set the config mode */
v_stat_s8 = bno055_get_operation_mode(&v_prev_opmode_u8);
if (v_stat_s8 == SUCCESS) {
	if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
		v_stat_s8 += bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);
		if (v_stat_s8 == SUCCESS) {
			/* Write page as one */
			v_pg_stat_s8 = bno055_write_page_id(PAGE_ONE);
			if (v_pg_stat_s8 == SUCCESS) {
				/* Write the value
				of gyro highrate y threshold*/
				com_rslt = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_GYRO_HIGHRATE_Y_THRES__REG,
				&v_data_u8r, BNO055_ONE_U8X);
				if (com_rslt == SUCCESS) {
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_GYRO_HIGHRATE_Y_THRES,
					v_gyro_highrate_y_thres_u8);
					com_rslt +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_GYRO_HIGHRATE_Y_THRES__REG,
					&v_data_u8r, BNO055_ONE_U8X);
				}
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	} else {
	com_rslt = ERROR;
	}
}
if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
	/* set the operation mode of
	previous operation mode*/
	com_rslt += bno055_set_operation_mode
	(v_prev_opmode_u8);
return com_rslt;
}
/*!
 *	@brief This API used to read gyro highrate y hysteresis
 *	from page one register from 0x1A bit 5 to 6
 *
 *	@param v_gyro_highrate_y_hyst_u8 : The value of gyro highrate y hysteresis
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note Gyro high rate hysteresis calculated by
 *
 *	using this (255 + 256 * v_gyro_highrate_y_hyst_u8) *4 LSB
 *
 *	The high rate value scales with the range setting
 *
 *  v_gyro_range_u8	  |	hysteresis		|     LSB
 * -----------------  | -------------   | ---------
 *     2000           |    62.26dps     |   1LSB
 *     1000           |    31.13dps     |   1LSB
 *     500            |    15.56dps     |   1LSB
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_y_hyst(
u8 *v_gyro_highrate_y_hyst_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X)  {
		return E_NULL_PTR;
		} else {
		/*condition check for page, gyro highrate y hysteresis is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			v_stat_s8 = bno055_write_page_id(PAGE_ONE);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ONE)) {
			/* Read the value of gyro highrate y hysteresis*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_HIGHRATE_Y_HYST__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_gyro_highrate_y_hyst_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_GYRO_HIGHRATE_Y_HYST);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to write gyro highrate y hysteresis
 *	from page one register from 0x1A bit 5 to 6
 *
 *	@param v_gyro_highrate_y_hyst_u8 : The value of gyro highrate y hysteresis
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note Gyro high rate hysteresis calculated by
 *
 *	using this (255 + 256 * v_gyro_highrate_y_hyst_u8) *4 LSB
 *
 *	The high rate value scales with the range setting
 *
 *  v_gyro_range_u8	  |	hysteresis		|     LSB
 * -----------------  | -------------   | ---------
 *     2000           |    62.26dps     |   1LSB
 *     1000           |    31.13dps     |   1LSB
 *     500            |    15.56dps     |   1LSB
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_y_hyst(
u8 v_gyro_highrate_y_hyst_u8)
{
BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 v_data_u8r = BNO055_ZERO_U8X;
s8 v_stat_s8 = ERROR;
u8 v_prev_opmode_u8 = OPERATION_MODE_CONFIG;
s8 v_pg_stat_s8 = ERROR;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X)  {
	return E_NULL_PTR;
} else {
/* The write operation effective only if the operation
mode is in config mode, this part of code is checking the
current operation mode and set the config mode */
v_stat_s8 = bno055_get_operation_mode(&v_prev_opmode_u8);
if (v_stat_s8 == SUCCESS) {
	if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
		v_stat_s8 += bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);
		if (v_stat_s8 == SUCCESS) {
			/* Write page as one */
			v_pg_stat_s8 = bno055_write_page_id(PAGE_ONE);
			if (v_pg_stat_s8 == SUCCESS) {
				/* Write the value of
				gyro highrate y hysteresis*/
				com_rslt = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_GYRO_HIGHRATE_Y_HYST__REG,
				&v_data_u8r, BNO055_ONE_U8X);
				if (com_rslt == SUCCESS) {
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_GYRO_HIGHRATE_Y_HYST,
					v_gyro_highrate_y_hyst_u8);
					com_rslt +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_GYRO_HIGHRATE_Y_HYST__REG,
					&v_data_u8r, BNO055_ONE_U8X);
				}
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	} else {
	com_rslt = ERROR;
	}
}
if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
	/* set the operation mode of
	previous operation mode*/
	com_rslt += bno055_set_operation_mode
	(v_prev_opmode_u8);
return com_rslt;
}
/*!
 *	@brief This API used to read gyro highrate y duration
 *	from page one register from 0x1B bit 0 to 7
 *
 *	@param v_gyro_highrate_y_durn_u8 : The value of gyro highrate y duration
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note Gyro highrate duration calculate by using the formula
 *
 *	(1 + v_gyro_highrate_y_durn_u8)*2.5ms
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_y_durn(
u8 *v_gyro_highrate_y_durn_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X)  {
		return E_NULL_PTR;
		} else {
		/*condition check for page, gyro highrate y duration is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			v_stat_s8 = bno055_write_page_id(PAGE_ONE);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ONE)) {
			/* Read the value of gyro highrate y duration*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_HIGHRATE_Y_DURN__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_gyro_highrate_y_durn_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_GYRO_HIGHRATE_Y_DURN);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to write gyro highrate y duration
 *	from page one register from 0x1B bit 0 to 7
 *
 *	@param v_gyro_highrate_y_durn_u8 : The value of gyro highrate y duration
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note Gyro highrate duration calculate by using the formula
 *
 *	(1 + v_gyro_highrate_y_durn_u8)*2.5ms
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_y_durn(
u8 v_gyro_highrate_y_durn_u8)
{
BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 v_data_u8r = BNO055_ZERO_U8X;
s8 v_stat_s8 = ERROR;
u8 v_prev_opmode_u8 = OPERATION_MODE_CONFIG;
s8 v_pg_stat_s8 = ERROR;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X)  {
	return E_NULL_PTR;
	} else {
/* The write operation effective only if the operation
mode is in config mode, this part of code is checking the
current operation mode and set the config mode */
v_stat_s8 = bno055_get_operation_mode(&v_prev_opmode_u8);
if (v_stat_s8 == SUCCESS) {
	if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
		v_stat_s8 += bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);

		if (v_stat_s8 == SUCCESS) {
			/* Write page as one */
			v_pg_stat_s8 = bno055_write_page_id(PAGE_ONE);
			if (v_pg_stat_s8 == SUCCESS) {
				/* Write the value
				of gyro highrate y duration*/
				com_rslt = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_GYRO_HIGHRATE_Y_DURN__REG,
				&v_data_u8r, BNO055_ONE_U8X);
				if (com_rslt == SUCCESS) {
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_GYRO_HIGHRATE_Y_DURN,
					v_gyro_highrate_y_durn_u8);
					com_rslt +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_GYRO_HIGHRATE_Y_DURN__REG,
					&v_data_u8r, BNO055_ONE_U8X);
				}
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	} else {
	com_rslt = ERROR;
	}
	}
	if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
		/* set the operation mode of
		previous operation mode*/
		com_rslt += bno055_set_operation_mode
		(v_prev_opmode_u8);
	return com_rslt;
}
/*!
 *	@brief This API used to read gyro highrate z threshold
 *	from page one register from 0x1C bit 0 to 4
 *
 *	@param v_gyro_highrate_z_thres_u8 : The value of gyro highrate z threshold
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note Gyro highrate threshold dependent on the
 *	selection of gyro range
 *
 *  v_gyro_range_u8	  |	threshold		|     LSB
 * -----------------  | -------------   | ---------
 *     2000           |    62.5dps      |   1LSB
 *     1000           |    31.25dps     |   1LSB
 *     500            |    15.625dps    |   1LSB
 *     125            |    7.8125dps    |   1LSB
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_z_thres(
u8 *v_gyro_highrate_z_thres_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X)  {
		return E_NULL_PTR;
		} else {
		/*condition check for page, gyro highrate z threshold is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			v_stat_s8 = bno055_write_page_id(PAGE_ONE);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ONE)) {
			/* Read the value of gyro highrate z threshold*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_HIGHRATE_Z_THRES__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_gyro_highrate_z_thres_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_GYRO_HIGHRATE_Z_THRES);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to write gyro highrate z threshold
 *	from page one register from 0x1C bit 0 to 4
 *
 *	@param v_gyro_highrate_z_thres_u8 : The value of gyro highrate z threshold
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note Gyro highrate threshold dependent on the
 *	selection of gyro range
 *
 *  v_gyro_range_u8	  |	threshold		|     LSB
 * -----------------  | -------------   | ---------
 *     2000           |    62.5dps      |   1LSB
 *     1000           |    31.25dps     |   1LSB
 *     500            |    15.625dps    |   1LSB
 *     125            |    7.8125dps    |   1LSB
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_z_thres(
u8 v_gyro_highrate_z_thres_u8)
{
BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 v_data_u8r = BNO055_ZERO_U8X;
s8 v_stat_s8 = ERROR;
u8 v_prev_opmode_u8 = OPERATION_MODE_CONFIG;
s8 v_pg_stat_s8 = ERROR;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X)  {
	return E_NULL_PTR;
} else {
/* The write operation effective only if the operation
mode is in config mode, this part of code is checking the
current operation mode and set the config mode */
v_stat_s8 = bno055_get_operation_mode(&v_prev_opmode_u8);
if (v_stat_s8 == SUCCESS) {
	if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
		v_stat_s8 += bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);
		if (v_stat_s8 == SUCCESS) {
			/* Write page as one */
			v_pg_stat_s8 = bno055_write_page_id(PAGE_ONE);
			if (v_pg_stat_s8 == SUCCESS) {
				/* Write the value
				of gyro highrate z threshold*/
				com_rslt = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_GYRO_HIGHRATE_Z_THRES__REG,
				&v_data_u8r, BNO055_ONE_U8X);
				if (com_rslt == SUCCESS) {
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_GYRO_HIGHRATE_Z_THRES,
					v_gyro_highrate_z_thres_u8);
					com_rslt +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_GYRO_HIGHRATE_Z_THRES__REG,
					&v_data_u8r, BNO055_ONE_U8X);
				}
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	} else {
	com_rslt = ERROR;
	}
}
if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
	/* set the operation mode of
	previous operation mode*/
	com_rslt += bno055_set_operation_mode
	(v_prev_opmode_u8);
return com_rslt;
}
/*!
 *	@brief This API used to read gyro highrate z hysteresis
 *	from page one register from 0x1C bit 5 to 6
 *
 *	@param v_gyro_highrate_z_hyst_u8 : The value of gyro highrate z hysteresis
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note Gyro high rate hysteresis calculated by
 *
 *	using this (255 + 256 * v_gyro_highrate_z_hyst_u8) *4 LSB
 *
 *	The high rate value scales with the range setting
 *
 *  v_gyro_range_u8	  |	 hysteresis		|     LSB
 * -----------------  | -------------   | ---------
 *     2000           |    62.26dps     |   1LSB
 *     1000           |    31.13dps     |   1LSB
 *     500            |    15.56dps     |   1LSB
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_z_hyst(
u8 *v_gyro_highrate_z_hyst_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X)  {
		return E_NULL_PTR;
		} else {
		/*condition check for page, gyro highrate z hysteresis is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			v_stat_s8 = bno055_write_page_id(PAGE_ONE);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ONE)) {
			/* Read the value of gyro highrate z hysteresis*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_HIGHRATE_Z_HYST__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_gyro_highrate_z_hyst_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_GYRO_HIGHRATE_Z_HYST);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to write gyro highrate z hysteresis
 *	from page one register from 0x1C bit 5 to 6
 *
 *	@param v_gyro_highrate_z_hyst_u8 : The value of gyro highrate z hysteresis
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note Gyro high rate hysteresis calculated by
 *
 *	using this (255 + 256 * v_gyro_highrate_z_hyst_u8) *4 LSB
 *
 *	The high rate value scales with the range setting
 *
 *  v_gyro_range_u8	  |	 hysteresis		|     LSB
 * -----------------  | -------------   | ---------
 *     2000           |    62.26dps     |   1LSB
 *     1000           |    31.13dps     |   1LSB
 *     500            |    15.56dps     |   1LSB
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_z_hyst(
u8 v_gyro_highrate_z_hyst_u8)
{
BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 v_data_u8r = BNO055_ZERO_U8X;
s8 v_stat_s8 = ERROR;
u8 v_prev_opmode_u8 = OPERATION_MODE_CONFIG;
s8 v_pg_stat_s8 = ERROR;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X)  {
	return E_NULL_PTR;
} else {
/* The write operation effective only if the operation
mode is in config mode, this part of code is checking the
current operation mode and set the config mode */
v_stat_s8 = bno055_get_operation_mode(&v_prev_opmode_u8);
if (v_stat_s8 == SUCCESS) {
	if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
		v_stat_s8 += bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);
		if (v_stat_s8 == SUCCESS) {
			/* Write page as one */
			v_pg_stat_s8 = bno055_write_page_id(PAGE_ONE);
			if (v_pg_stat_s8 == SUCCESS) {
				/* Write the value
				of gyro highrate z hysteresis*/
				com_rslt = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_GYRO_HIGHRATE_Z_HYST__REG,
				&v_data_u8r, BNO055_ONE_U8X);
				if (com_rslt == SUCCESS) {
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_GYRO_HIGHRATE_Z_HYST,
					v_gyro_highrate_z_hyst_u8);
					com_rslt +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_GYRO_HIGHRATE_Z_HYST__REG,
					&v_data_u8r, BNO055_ONE_U8X);
				}
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	} else {
	com_rslt = ERROR;
	}
}
if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
	/* set the operation mode of
	previous operation mode*/
	com_rslt += bno055_set_operation_mode
	(v_prev_opmode_u8);
return com_rslt;
}
/*!
 *	@brief This API used to read gyro highrate z duration
 *	from page one register from 0x1D bit 0 to 7
 *
 *	@param v_gyro_highrate_z_durn_u8 : The value of gyro highrate z duration
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note Gyro highrate duration calculate by using the formula
 *
 *	(1 + v_gyro_highrate_z_durn_u8)*2.5ms
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_z_durn(
u8 *v_gyro_highrate_z_durn_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X)  {
		return E_NULL_PTR;
		} else {
		/*condition check for page, gyro highrate z duration is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			v_stat_s8 = bno055_write_page_id(PAGE_ONE);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ONE)) {
			/* Read the value of gyro highrate z duration*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_HIGHRATE_Z_DURN__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_gyro_highrate_z_durn_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_GYRO_HIGHRATE_Z_DURN);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to write gyro highrate z duration
 *	from page one register from 0x1D bit 0 to 7
 *
 *	@param v_gyro_highrate_z_durn_u8 : The value of gyro highrate z duration
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note Gyro highrate duration calculate by using the formula
 *
 *	(1 + v_gyro_highrate_z_durn_u8)*2.5ms
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_z_durn(
u8 v_gyro_highrate_z_durn_u8)
{
BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 v_data_u8r = BNO055_ZERO_U8X;
s8 v_stat_s8 = ERROR;
s8 v_pg_stat_s8 = ERROR;
u8 v_prev_opmode_u8 = OPERATION_MODE_CONFIG;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X)  {
	return E_NULL_PTR;
} else {
/* The write operation effective only if the operation
mode is in config mode, this part of code is checking the
current operation mode and set the config mode */
v_stat_s8 = bno055_get_operation_mode(&v_prev_opmode_u8);
if (v_stat_s8 == SUCCESS) {
	if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
		v_stat_s8 += bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);
		if (v_stat_s8 == SUCCESS) {
			/* Write page as one */
			v_pg_stat_s8 = bno055_write_page_id(PAGE_ONE);
			if (v_pg_stat_s8 == SUCCESS) {
				/* Write the value of
				gyro highrate z duration*/
				com_rslt = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_GYRO_HIGHRATE_Z_DURN__REG,
				&v_data_u8r, BNO055_ONE_U8X);
				if (com_rslt == SUCCESS) {
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_GYRO_HIGHRATE_Z_DURN,
					v_gyro_highrate_z_durn_u8);
					com_rslt +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_GYRO_HIGHRATE_Z_DURN__REG,
					&v_data_u8r, BNO055_ONE_U8X);
				}
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	} else {
	com_rslt = ERROR;
	}
	}
	if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
		/* set the operation mode of
		previous operation mode*/
		com_rslt += bno055_set_operation_mode
		(v_prev_opmode_u8);
	return com_rslt;
}
/*!
 *	@brief This API used to read gyro anymotion threshold
 *	from page one register from 0x1E bit 0 to 6
 *
 *	@param v_gyro_any_motion_thres_u8 : The value of gyro anymotion threshold
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note Gyro anymotion interrupt threshold dependent
 *	on the selection of gyro range
 *
 *  v_gyro_range_u8	  |	threshold	  |	   LSB
 * -----------------  | ------------- | ---------
 *     2000           |    1dps       |   1LSB
 *     1000           |    0.5dps     |   1LSB
 *     500            |    0.25dps    |   1LSB
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_any_motion_thres(
u8 *v_gyro_any_motion_thres_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X)  {
		return E_NULL_PTR;
		} else {
		/*condition check for page,gyro anymotion threshold is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			v_stat_s8 = bno055_write_page_id(PAGE_ONE);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ONE)) {
			/* Read the value of gyro anymotion threshold*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_ANY_MOTION_THRES__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_gyro_any_motion_thres_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_GYRO_ANY_MOTION_THRES);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to write gyro anymotion threshold
 *	from page one register from 0x1E bit 0 to 6
 *
 *	@param v_gyro_any_motion_thres_u8 : The value of gyro anymotion threshold
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note Gyro anymotion interrupt threshold dependent
 *	on the selection of gyro range
 *
 *  v_gyro_range_u8	  |	threshold	  |	   LSB
 * -----------------  | ------------- | ---------
 *     2000           |    1dps       |   1LSB
 *     1000           |    0.5dps     |   1LSB
 *     500            |    0.25dps    |   1LSB
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_any_motion_thres(
u8 v_gyro_any_motion_thres_u8)
{
BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 v_data_u8r = BNO055_ZERO_U8X;
s8 v_stat_s8 = ERROR;
u8 v_prev_opmode_u8 = OPERATION_MODE_CONFIG;
s8 v_pg_stat_s8 = ERROR;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X)  {
	return E_NULL_PTR;
} else {
/* The write operation effective only if the operation
mode is in config mode, this part of code is checking the
current operation mode and set the config mode */
v_stat_s8 = bno055_get_operation_mode(&v_prev_opmode_u8);
if (v_stat_s8 == SUCCESS) {
	if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
		v_stat_s8 += bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);
		if (v_stat_s8 == SUCCESS) {
			/* Write page as one */
			v_pg_stat_s8 = bno055_write_page_id(PAGE_ONE);
			if (v_pg_stat_s8 == SUCCESS) {
				/* Write the value
				of gyro anymotion threshold*/
				com_rslt = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_GYRO_ANY_MOTION_THRES__REG,
				&v_data_u8r, BNO055_ONE_U8X);
				if (com_rslt == SUCCESS) {
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_GYRO_ANY_MOTION_THRES,
					v_gyro_any_motion_thres_u8);
					com_rslt +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_GYRO_ANY_MOTION_THRES__REG,
					&v_data_u8r, BNO055_ONE_U8X);
				}
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	} else {
	com_rslt = ERROR;
	}
}
if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
	/* set the operation mode of
	previous operation mode*/
	com_rslt += bno055_set_operation_mode
	(v_prev_opmode_u8);
return com_rslt;
}
/*!
 *	@brief This API used to read gyro anymotion slope samples
 *	from page one register from 0x1F bit 0 to 1
 *
 *	@param v_gyro_any_motion_slope_samples_u8 :
 *	The value of gyro anymotion slope samples
 *  v_gyro_any_motion_slope_samples_u8   |   result
 *  ----------------------------------   | -----------
 *            0                          |    8 samples
 *            1                          |    16 samples
 *            2                          |    32 samples
 *            3                          |    64 samples
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_any_motion_slope_samples(
u8 *v_gyro_any_motion_slope_samples_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X)  {
		return E_NULL_PTR;
		} else {
		/*condition check for page, gyro anymotion slope samples is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			v_stat_s8 = bno055_write_page_id(PAGE_ONE);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ONE)) {
			/*Read the value of gyro anymotion slope samples*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_SLOPE_SAMPLES__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_gyro_any_motion_slope_samples_u8 =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_GYRO_SLOPE_SAMPLES);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to write gyro anymotion slope samples
 *	from page one register from 0x1F bit 0 to 1
 *
 *	@param v_gyro_any_motion_slope_samples_u8 :
 *	The value of gyro anymotion slope samples
 *  v_gyro_any_motion_slope_samples_u8   |   result
 *  ----------------------------------   | -----------
 *            0                          |    8 samples
 *            1                          |    16 samples
 *            2                          |    32 samples
 *            3                          |    64 samples
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_any_motion_slope_samples(
u8 v_gyro_any_motion_slope_samples_u8)
{
BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 v_data_u8r = BNO055_ZERO_U8X;
s8 v_stat_s8 = ERROR;
s8 v_pg_stat_s8 = ERROR;
u8 v_prev_opmode_u8 = OPERATION_MODE_CONFIG;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X)  {
	return E_NULL_PTR;
} else {
/* The write operation effective only if the operation
mode is in config mode, this part of code is checking the
current operation mode and set the config mode */
v_stat_s8 = bno055_get_operation_mode(&v_prev_opmode_u8);
if (v_stat_s8 == SUCCESS) {
	if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
		v_stat_s8 += bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);
		if (v_stat_s8 == SUCCESS) {
			/* Write page as one */
			v_pg_stat_s8 = bno055_write_page_id(PAGE_ONE);
			if (v_pg_stat_s8 == SUCCESS) {
				/* Write the value of
				gyro anymotion slope samples*/
				com_rslt = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_GYRO_SLOPE_SAMPLES__REG,
				&v_data_u8r, BNO055_ONE_U8X);
				if (com_rslt == SUCCESS) {
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_GYRO_SLOPE_SAMPLES,
					v_gyro_any_motion_slope_samples_u8);
					com_rslt +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_GYRO_SLOPE_SAMPLES__REG,
					&v_data_u8r, BNO055_ONE_U8X);
				}
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	} else {
	com_rslt = ERROR;
	}
	}
	if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
		/* set the operation mode of
		previous operation mode*/
		com_rslt += bno055_set_operation_mode
		(v_prev_opmode_u8);
	return com_rslt;
}
/*!
 *	@brief This API used to read gyro anymotion awake duration
 *	from page one register from 0x1F bit 2 to 3
 *
 *	@param v_gyro_awake_durn_u8 : The value of gyro anymotion awake duration
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_any_motion_awake_durn(
u8 *v_gyro_awake_durn_u8)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8r = BNO055_ZERO_U8X;
	s8 v_stat_s8 = ERROR;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X)  {
		return E_NULL_PTR;
		} else {
		/*condition check for page, gyro anymotion awake duration is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			v_stat_s8 = bno055_write_page_id(PAGE_ONE);
		if ((v_stat_s8 == SUCCESS) ||
		(p_bno055->page_id == PAGE_ONE)) {
			/* Read the value of gyro anymotion awake duration*/
			com_rslt = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_AWAKE_DURN__REG,
			&v_data_u8r, BNO055_ONE_U8X);
			*v_gyro_awake_durn_u8 = BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_GYRO_AWAKE_DURN);
		} else {
		com_rslt = ERROR;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API used to write gyro anymotion awake duration
 *	from page one register from 0x1F bit 2 to 3
 *
 *	@param v_gyro_awake_durn_u8 : The value of gyro anymotion awake duration
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_any_motion_awake_durn(
u8 v_gyro_awake_durn_u8)
{
BNO055_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 v_data_u8r = BNO055_ZERO_U8X;
s8 v_stat_s8 = ERROR;
s8 v_pg_stat_s8 = ERROR;
u8 v_prev_opmode_u8 = OPERATION_MODE_CONFIG;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X)  {
	return E_NULL_PTR;
	} else {
	/* The write operation effective only if the operation
	mode is in config mode, this part of code is checking the
	current operation mode and set the config mode */
	v_stat_s8 = bno055_get_operation_mode(&v_prev_opmode_u8);
	if (v_stat_s8 == SUCCESS) {
		if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
			v_stat_s8 += bno055_set_operation_mode
			(OPERATION_MODE_CONFIG);
			if (v_stat_s8 == SUCCESS) {
				/* Write page as one */
				v_pg_stat_s8 = bno055_write_page_id(PAGE_ONE);
				if (v_pg_stat_s8 == SUCCESS) {
					/* Write the value of gyro
					anymotion awake duration*/
					com_rslt =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_GYRO_AWAKE_DURN__REG,
					&v_data_u8r, BNO055_ONE_U8X);
					if (com_rslt == SUCCESS) {
						v_data_u8r =
						BNO055_SET_BITSLICE(v_data_u8r,
						BNO055_GYRO_AWAKE_DURN,
						v_gyro_awake_durn_u8);
						com_rslt +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_GYRO_AWAKE_DURN__REG,
						&v_data_u8r, BNO055_ONE_U8X);
					}
				} else {
				com_rslt = ERROR;
				}
			} else {
			com_rslt = ERROR;
			}
		} else {
		com_rslt = ERROR;
		}
	}
	if (v_prev_opmode_u8 != OPERATION_MODE_CONFIG)
		/* set the operation mode
		of previous operation mode*/
		com_rslt += bno055_set_operation_mode
		(v_prev_opmode_u8);
	return com_rslt;
}
