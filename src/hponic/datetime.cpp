#include "datetime.h"

static struct datetime current_datetime;

void datetime_set(struct datetime *dt)
{
	current_datetime = *dt;
}

struct datetime datetime_now(void)
{
	return current_datetime;
}

static bool datetime_day_ge(struct datetime *dt, struct datetime *point)
{
	if (dt->hours < point->hours)
		return false;
	if (dt->minutes < point->minutes)
		return false;
	if (dt->seconds < point->seconds)
		return false;
	return true;
}

static bool datetime_month_ge(struct datetime *dt, struct datetime *point)
{
	if (dt->day < point->day)
		return false;
	return datetime_day_ge(dt, point);
}

static bool datetime_year_ge(struct datetime *dt, struct datetime *point)
{
	if (dt->month < point->month)
		return false;
	return datetime_month_ge(dt, point);
}

static bool datetime_strict_ge(struct datetime *dt, struct datetime *point)
{
	if (dt->year < point->year)
		return false;
	return datetime_year_ge(dt, point);
}

static bool datetime_day_le(struct datetime *dt, struct datetime *point)
{
	if (dt->hours > point->hours)
		return false;
	if (dt->minutes > point->minutes)
		return false;
	if (dt->seconds > point->seconds)
		return false;
	return true;
}

static bool datetime_month_le(struct datetime *dt, struct datetime *point)
{
	if (dt->day > point->day)
		return false;
	return datetime_day_le(dt, point);
}

static bool datetime_year_le(struct datetime *dt, struct datetime *point)
{
	if (dt->month > point->month)
		return false;
	return datetime_month_le(dt, point);
}

static bool datetime_strict_le(struct datetime *dt, struct datetime *point)
{
	if (dt->year > point->year)
		return false;
	if (dt->month > point->month)
		return false;
	if (dt->day > point->day)
		return false;
	if (dt->hours > point->hours)
		return false;
	if (dt->minutes > point->minutes)
		return false;
	if (dt->seconds > point->seconds)
		return false;
	return true;
}

bool datetime_in(	struct datetime *dt, 
					struct datetime *point1, struct datetime *point2, 
					enum time_constrains constrains)
{
	switch (constrains)
	{
	case ALL_TIME:
		return true;
	case STRICT_EQUALITY:
		return (datetime_strict_ge(dt, point1) && datetime_strict_le(dt, point2));
	case EVERY_DAY
	}
}
