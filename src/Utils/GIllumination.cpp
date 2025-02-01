/*
 * Copyright (C) 2018 Erion Plaku
 * All Rights Reserved
 * 
 *       Created by Erion Plaku
 *       Computational Robotics Group
 *       Department of Electrical Engineering and Computer Science
 *       Catholic University of America
 *
 *       www.robotmotionplanning.org
 *
 * Code should not be distributed or used without written permission from the
 * copyright holder.
 */
#include "Utils/GIllumination.hpp"

namespace Antipatrea
{
void GIllumination::CopyFrom(const GIllumination &gIllum)
{
	GProperties::CopyFrom(gIllum);
	RemoveLights();
	for (int i = 0; i < gIllum.GetNrLights(); ++i)
	{
		GLight *glight = new GLight();
		glight->CopyFrom(*(gIllum.GetLight(i)));
		AddLight(glight);
	}
}

void GIllumination::AddDefaultLights(void)
{
	GLight *light;
	const double h = 100.0;

	light = new GLight();
	light->SetPosition(200.0, 200.0, h);
	AddLight(light);

	light = new GLight();
	light->SetPosition(-200.0, 200.0, h);
	AddLight(light);

	light = new GLight();
	light->SetPosition(200.0, -200.0, h);
	AddLight(light);

	light = new GLight();
	light->SetPosition(-200.0, -200.0, h);
	AddLight(light);

	//	    light = new GLight();
	//	    light->SetPosition(0.0, 0.0, h);
	//	    AddLight(light);
}

void GIllumination::SetupFromParams(Params &params)
{
	std::string kw;

	GProperties::SetupFromParams(params);

	const int n = params.GetValueAsInt(Constants::KW_NrLights, Constants::ID_UNDEFINED);
	if (n <= 0)
	{
		RemoveLights();
		return;
	}

	for (int i = 0; i < n; ++i)
		AddLight(new GLight());
	for (int i = 0; i < n; ++i)
	{
		kw = Constants::KW_Light;
		kw = kw + std::to_string(i);
		auto data = params.GetData(kw.c_str());
		if (data && data->m_params)
			GetLight(i)->SetupFromParams(*(data->m_params));
	}
}
}
