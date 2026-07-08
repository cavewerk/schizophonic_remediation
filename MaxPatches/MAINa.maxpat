{
	"patcher" : 	{
		"fileversion" : 1,
		"appversion" : 		{
			"major" : 9,
			"minor" : 0,
			"revision" : 7,
			"architecture" : "x64",
			"modernui" : 1
		}
,
		"classnamespace" : "box",
		"rect" : [ 74.0, 94.0, 890.0, 757.0 ],
		"openinpresentation" : 1,
		"gridsize" : [ 15.0, 15.0 ],
		"style" : "chiba",
		"boxes" : [ 			{
				"box" : 				{
					"fontface" : 0,
					"fontname" : "Arial",
					"fontsize" : 12.0,
					"id" : "obj-136",
					"maxclass" : "number~",
					"mode" : 2,
					"numinlets" : 2,
					"numoutlets" : 2,
					"outlettype" : [ "signal", "float" ],
					"patching_rect" : [ 1043.35128116607666, 757.142023921012878, 56.0, 22.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 26.66666579246521, 919.918857574462891, 75.325607270002365, 22.0 ],
					"sig" : 0.0
				}

			}
, 			{
				"box" : 				{
					"format" : 6,
					"id" : "obj-161",
					"maxclass" : "flonum",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 365.040650188922882, 455.284552574157715, 50.0, 22.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 365.040650188922882, 455.284552574157715, 50.0, 22.0 ]
				}

			}
, 			{
				"box" : 				{
					"format" : 6,
					"id" : "obj-248",
					"maxclass" : "flonum",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 538.732401430606842, 233.802819967269897, 50.0, 22.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-246",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 360.563385009765625, 219.718312740325928, 147.0, 22.0 ],
					"text" : "prepend HarmModTouch0"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-245",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 309.310000000000002, 240.860000000000014, 32.0, 22.0 ],
					"text" : "print"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-244",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 87.323944807052612, 186.0, 50.704226016998291, 22.0 ],
					"text" : "in4 $1"
				}

			}
, 			{
				"box" : 				{
					"fontface" : 0,
					"fontname" : "Arial",
					"fontsize" : 12.0,
					"id" : "obj-240",
					"maxclass" : "number~",
					"mode" : 2,
					"numinlets" : 2,
					"numoutlets" : 2,
					"outlettype" : [ "signal", "float" ],
					"patching_rect" : [ 213.5, 360.065639138221741, 56.0, 22.0 ],
					"sig" : 0.0
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-239",
					"linecount" : 2,
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 87.323944807052612, 209.859157681465149, 83.098592638969421, 35.0 ],
					"text" : "pattrforward Osc0"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-238",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 225.352115631103516, 400.00000524520874, 100.0, 22.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 225.352115631103516, 400.00000524520874, 32.0, 22.0 ],
					"text" : "pvar"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-237",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 217.870000000000005, 486.110000000000014, 100.0, 22.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 223.423591956496239, 421.830991446971893, 73.0, 22.0 ],
					"text" : "pattrforward"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-236",
					"maxclass" : "live.numbox",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 416.666656732559204, 529.999987363815308, 44.0, 15.0 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_longname" : "live.numbox[5]",
							"parameter_modmode" : 3,
							"parameter_shortname" : "live.numbox",
							"parameter_type" : 0,
							"parameter_unitstyle" : 0
						}

					}
,
					"varname" : "live.numbox[5]"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-235",
					"maxclass" : "live.numbox",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 795.83331435918808, 382.499990880489349, 44.0, 15.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 172.40000256896019, 321.60000479221344, 35.494741708040237, 15.0 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_longname" : "live.numbox[4]",
							"parameter_modmode" : 3,
							"parameter_shortname" : "live.numbox",
							"parameter_type" : 0,
							"parameter_unitstyle" : 0
						}

					}
,
					"varname" : "live.numbox[4]"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-234",
					"maxclass" : "live.numbox",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 659.525307774543762, 379.44936215877533, 44.0, 15.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 132.000003933906555, 321.666676253080368, 36.421054720878601, 15.0 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_longname" : "live.numbox[3]",
							"parameter_modmode" : 3,
							"parameter_shortname" : "live.numbox",
							"parameter_type" : 0,
							"parameter_unitstyle" : 0
						}

					}
,
					"varname" : "live.numbox[3]"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-233",
					"maxclass" : "live.numbox",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 533.911385416984558, 379.44936215877533, 44.0, 15.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 91.869918644428253, 321.544715255498886, 37.077454388141632, 15.0 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_longname" : "live.numbox[2]",
							"parameter_modmode" : 3,
							"parameter_shortname" : "live.numbox",
							"parameter_type" : 0,
							"parameter_unitstyle" : 0
						}

					}
,
					"varname" : "live.numbox[2]"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-231",
					"maxclass" : "live.numbox",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 414.924045205116272, 373.4177166223526, 44.0, 15.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 53.378374814987183, 321.621600151062012, 35.810808420181274, 15.0 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_longname" : "live.numbox[1]",
							"parameter_modmode" : 3,
							"parameter_shortname" : "live.numbox",
							"parameter_type" : 0,
							"parameter_unitstyle" : 0
						}

					}
,
					"varname" : "live.numbox[1]"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-230",
					"maxclass" : "live.numbox",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 275.0, 360.065639138221741, 44.0, 15.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 13.255523160099983, 321.500529617071152, 38.142854183912277, 15.0 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_longname" : "live.numbox",
							"parameter_modmode" : 3,
							"parameter_shortname" : "live.numbox",
							"parameter_type" : 0,
							"parameter_unitstyle" : 0
						}

					}
,
					"varname" : "live.numbox"
				}

			}
, 			{
				"box" : 				{
					"fontface" : 0,
					"fontname" : "Arial",
					"fontsize" : 12.0,
					"id" : "obj-229",
					"maxclass" : "number~",
					"mode" : 2,
					"numinlets" : 2,
					"numoutlets" : 2,
					"outlettype" : [ "signal", "float" ],
					"patching_rect" : [ 30.450338304042816, 387.0, 54.584169507026672, 22.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 10.223793268203735, 410.0, 54.584169507026672, 22.0 ],
					"sig" : 199.128009693770423
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-220",
					"maxclass" : "ezdac~",
					"numinlets" : 2,
					"numoutlets" : 0,
					"patching_rect" : [ 50.576930046081543, 797.959176063537598, 45.0, 45.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-21",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 760.317472100257874, 1220.634939551353455, 35.0, 22.0 ],
					"text" : "clear"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-23",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 673.015883445739746, 1193.650812149047852, 47.0, 22.0 ],
					"text" : "gridList"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-27",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 673.015883445739746, 1166.666684746742249, 104.0, 22.0 ],
					"text" : "udpreceive 30338"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-31",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 849.206362366676331, 1503.174626469612122, 41.0, 22.0 ],
					"text" : "s st8a"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-42",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 804.761917233467102, 1503.174626469612122, 41.0, 22.0 ],
					"text" : "s st7a"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-44",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 760.317472100257874, 1503.174626469612122, 41.0, 22.0 ],
					"text" : "s st6a"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-47",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 714.285725355148315, 1503.174626469612122, 41.0, 22.0 ],
					"text" : "s st5a"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-48",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 661.904772162437439, 1503.174626469612122, 41.0, 22.0 ],
					"text" : "s st4a"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-49",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 614.285723805427551, 1503.174626469612122, 41.0, 22.0 ],
					"text" : "s st3a"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-72",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 578.481005072593689, 1503.797448635101318, 41.0, 22.0 ],
					"text" : "s st2a"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-83",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 533.333341598510742, 1503.174626469612122, 41.0, 22.0 ],
					"text" : "s st1a"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-90",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 496.825404524803162, 1503.174626469612122, 41.0, 22.0 ],
					"text" : "s st0a"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-118",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 560.317469000816345, 1212.698431491851807, 42.0, 22.0 ],
					"text" : "$1 0 1"
				}

			}
, 			{
				"box" : 				{
					"autosize" : 1,
					"columns" : 16,
					"dialmode" : 1,
					"horizontalspacing" : 4,
					"id" : "obj-119",
					"maxclass" : "matrixctrl",
					"numinlets" : 1,
					"numoutlets" : 2,
					"one/column" : 1,
					"one/matrix" : 1,
					"one/row" : 1,
					"outlettype" : [ "list", "list" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 504.76191258430481, 1441.269863605499268, 322.0, 22.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 518.987334966659546, 740.506319403648376, 322.0, 22.0 ],
					"rows" : 1,
					"style" : "rnbohighcontrast",
					"verticalspacing" : 4
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-126",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 9,
					"outlettype" : [ "float", "float", "float", "float", "float", "float", "float", "float", "float" ],
					"patching_rect" : [ 504.76191258430481, 1473.015895843505859, 363.0, 22.0 ],
					"text" : "unpack f f f f f f f f f"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-171",
					"maxclass" : "number",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 501.58730936050415, 1212.698431491851807, 50.0, 22.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-174",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 3,
					"outlettype" : [ "signal", "signal", "int" ],
					"patching_rect" : [ 504.76191258430481, 1182.539700865745544, 67.0, 22.0 ],
					"text" : "subdiv~ 16"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-208",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "signal" ],
					"patching_rect" : [ 504.76191258430481, 1153.968271851539612, 116.0, 22.0 ],
					"text" : "phasor~ 1n @lock 1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-210",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 673.015883445739746, 1220.634939551353455, 81.0, 22.0 ],
					"text" : "getcolumn $1"
				}

			}
, 			{
				"box" : 				{
					"autosize" : 1,
					"columns" : 16,
					"dialmode" : 2,
					"horizontalspacing" : 4,
					"id" : "obj-212",
					"maxclass" : "matrixctrl",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "list", "list" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 504.76191258430481, 1253.968273401260376, 322.0, 182.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 518.987334966659546, 554.430372476577759, 322.0, 182.0 ],
					"rows" : 9,
					"style" : "rnbohighcontrast",
					"verticalspacing" : 4
				}

			}
, 			{
				"box" : 				{
					"angle" : 270.0,
					"bgcolor" : [ 0.494117647058824, 0.780392156862745, 0.796078431372549, 1.0 ],
					"id" : "obj-219",
					"maxclass" : "panel",
					"mode" : 0,
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 490.476198077201843, 1136.507954120635986, 401.234599947929382, 404.93830394744873 ],
					"proportion" : 0.39
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-5",
					"maxclass" : "newobj",
					"numinlets" : 9,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patcher" : 					{
						"fileversion" : 1,
						"appversion" : 						{
							"major" : 9,
							"minor" : 0,
							"revision" : 7,
							"architecture" : "x64",
							"modernui" : 1
						}
,
						"classnamespace" : "box",
						"rect" : [ 436.0, 183.0, 1000.0, 739.0 ],
						"gridsize" : [ 15.0, 15.0 ],
						"boxes" : [ 							{
								"box" : 								{
									"id" : "obj-17",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 724.742227435112, 98.400001466274261, 131.0, 22.0 ],
									"text" : "prepend /harmLabelA8"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-16",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 641.237077474594116, 98.400001466274261, 131.0, 22.0 ],
									"text" : "prepend /harmLabelA7"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-15",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 556.700999736785889, 98.400001466274261, 131.0, 22.0 ],
									"text" : "prepend /harmLabelA6"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-14",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 472.164921998977661, 98.400001466274261, 131.0, 22.0 ],
									"text" : "prepend /harmLabelA5"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-13",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 387.628844261169434, 98.400001466274261, 131.0, 22.0 ],
									"text" : "prepend /harmLabelA4"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-12",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 303.092766523361206, 98.400001466274261, 131.0, 22.0 ],
									"text" : "prepend /harmLabelA3"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-11",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 218.556688785552979, 98.400001466274261, 131.0, 22.0 ],
									"text" : "prepend /harmLabelA2"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-10",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 134.020611047744751, 98.400001466274261, 131.0, 22.0 ],
									"text" : "prepend /harmLabelA1"
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-9",
									"index" : 9,
									"maxclass" : "inlet",
									"numinlets" : 0,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 723.711299657821655, 57.731955528259277, 23.0, 23.0 ]
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-8",
									"index" : 8,
									"maxclass" : "inlet",
									"numinlets" : 0,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 639.497387200593948, 58.0, 23.0, 23.0 ]
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-7",
									"index" : 7,
									"maxclass" : "inlet",
									"numinlets" : 0,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 555.283474743366241, 58.0, 23.0, 23.0 ]
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-6",
									"index" : 6,
									"maxclass" : "inlet",
									"numinlets" : 0,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 471.069562286138535, 58.0, 23.0, 23.0 ]
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-5",
									"index" : 5,
									"maxclass" : "inlet",
									"numinlets" : 0,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 386.855649828910828, 58.0, 23.0, 23.0 ]
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-4",
									"index" : 4,
									"maxclass" : "inlet",
									"numinlets" : 0,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 302.641737371683121, 58.0, 23.0, 23.0 ]
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-3",
									"index" : 3,
									"maxclass" : "inlet",
									"numinlets" : 0,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 218.427824914455414, 58.0, 23.0, 23.0 ]
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-2",
									"index" : 2,
									"maxclass" : "inlet",
									"numinlets" : 0,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 134.213912457227707, 58.0, 23.0, 23.0 ]
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-1",
									"index" : 1,
									"maxclass" : "inlet",
									"numinlets" : 0,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 50.0, 58.0, 23.0, 23.0 ]
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-227",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 50.0, 98.400001466274261, 131.0, 22.0 ],
									"text" : "prepend /harmLabelA0"
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-231",
									"index" : 1,
									"maxclass" : "outlet",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 50.0, 210.0, 30.0, 30.0 ]
								}

							}
 ],
						"lines" : [ 							{
								"patchline" : 								{
									"destination" : [ "obj-227", 0 ],
									"source" : [ "obj-1", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-231", 0 ],
									"source" : [ "obj-10", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-231", 0 ],
									"source" : [ "obj-11", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-231", 0 ],
									"source" : [ "obj-12", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-231", 0 ],
									"source" : [ "obj-13", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-231", 0 ],
									"source" : [ "obj-14", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-231", 0 ],
									"source" : [ "obj-15", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-231", 0 ],
									"source" : [ "obj-16", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-231", 0 ],
									"source" : [ "obj-17", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-10", 0 ],
									"source" : [ "obj-2", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-231", 0 ],
									"source" : [ "obj-227", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-11", 0 ],
									"source" : [ "obj-3", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-12", 0 ],
									"source" : [ "obj-4", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-13", 0 ],
									"source" : [ "obj-5", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-14", 0 ],
									"source" : [ "obj-6", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-15", 0 ],
									"source" : [ "obj-7", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-16", 0 ],
									"source" : [ "obj-8", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-17", 0 ],
									"source" : [ "obj-9", 0 ]
								}

							}
 ]
					}
,
					"patching_rect" : [ 1227.887527942657471, 568.000016927719116, 186.714287281036377, 22.0 ],
					"text" : "p oscSend"
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-432",
					"maxclass" : "slider",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1072.340417861938477, 51.06382942199707, 11.0, 61.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 108.108100891113281, 881.081022262573242, 27.90697717666626, 61.240311026573181 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_longname" : "ampFundA",
							"parameter_mmax" : 1.0,
							"parameter_modmode" : 3,
							"parameter_shortname" : "slider[1]",
							"parameter_type" : 0
						}

					}
,
					"size" : 1.0,
					"varname" : "slider[21]"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-433",
					"linecount" : 4,
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 1072.340417861938477, 117.021275758743286, 30.434782862663269, 62.0 ],
					"text" : "s ampFunda"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-401",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1051.35128116607666, 810.810756683349609, 115.0, 22.0 ],
					"text" : "prepend /fundFreqA"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-402",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 1051.35128116607666, 644.594551563262939, 24.0, 24.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 140.540531158447266, 918.918857574462891, 24.0, 24.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-403",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1339.965232968330383, 677.871577978134155, 46.0, 22.0 ],
					"text" : "r funda"
				}

			}
, 			{
				"box" : 				{
					"bgcolor" : [ 0.125490196078431, 0.125490196078431, 0.125490196078431, 1.0 ],
					"elementcolor" : [ 0.717647058823529, 0.831372549019608, 0.572549019607843, 1.0 ],
					"floatoutput" : 1,
					"id" : "obj-404",
					"maxclass" : "slider",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1256.756672859191895, 643.243200302124023, 20.0, 140.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 398.648622035980225, 537.837801933288574, 53.246752738952637, 329.792204856872559 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_longname" : "mainAmpA",
							"parameter_mmax" : 1.0,
							"parameter_modmode" : 3,
							"parameter_shortname" : "slider",
							"parameter_type" : 0
						}

					}
,
					"size" : 1.0,
					"varname" : "slider[20]"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-405",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 1056.756686210632324, 785.135082721710205, 48.0, 22.0 ],
					"text" : "s funda"
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-406",
					"maxclass" : "dial",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1051.35128116607666, 675.675630569458008, 40.0, 40.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 27.02702522277832, 881.081022262573242, 36.082472205162048, 36.082472205162048 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_longname" : "fundCoarseA",
							"parameter_mmax" : 1.0,
							"parameter_modmode" : 3,
							"parameter_shortname" : "dial",
							"parameter_type" : 0
						}

					}
,
					"size" : 1.0,
					"varname" : "dial[20]"
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-407",
					"maxclass" : "dial",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1099.999926567077637, 675.675630569458008, 40.0, 40.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 66.216211795806885, 881.081022262573242, 36.082472205162048, 36.082472205162048 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_longname" : "fundFineA",
							"parameter_mmax" : 1.0,
							"parameter_modmode" : 3,
							"parameter_shortname" : "dial",
							"parameter_type" : 0
						}

					}
,
					"size" : 1.0,
					"varname" : "dial[21]"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-409",
					"maxclass" : "newobj",
					"numinlets" : 3,
					"numoutlets" : 2,
					"outlettype" : [ "signal", "signal" ],
					"patching_rect" : [ 1051.35128116607666, 729.729681015014648, 50.0, 22.0 ],
					"text" : "fundSet"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-410",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1114.00003319978714, 725.844639182090759, 73.0, 22.0 ],
					"text" : "r ampFunda"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-411",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"patching_rect" : [ 1082.432360172271729, 644.594551563262939, 58.0, 22.0 ],
					"text" : "loadbang"
				}

			}
, 			{
				"box" : 				{
					"dbperled" : 1,
					"id" : "obj-412",
					"maxclass" : "meter~",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"patching_rect" : [ 1218.918837547302246, 825.675620555877686, 77.0, 22.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 459.459428787231445, 537.837801933288574, 44.155843734741211, 329.870126724243164 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-413",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "signal" ],
					"patching_rect" : [ 1218.918837547302246, 762.162111282348633, 34.0, 22.0 ],
					"text" : "*~ 1."
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-414",
					"maxclass" : "newobj",
					"numinlets" : 3,
					"numoutlets" : 1,
					"outlettype" : [ "signal" ],
					"patching_rect" : [ 1218.918837547302246, 791.891839027404785, 60.0, 22.0 ],
					"text" : "clip~ 0. 1."
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-415",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "float", "float" ],
					"patching_rect" : [ 1339.965232968330383, 704.954910278320312, 39.0, 22.0 ],
					"text" : "t f f"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-416",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"patching_rect" : [ 1387.881897807121277, 646.621579170227051, 58.0, 22.0 ],
					"text" : "loadbang"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-417",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 4,
					"outlettype" : [ "int", "float", "int", "int" ],
					"patching_rect" : [ 1387.881897807121277, 675.788244724273682, 61.0, 22.0 ],
					"text" : "dspstate~"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-418",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"patching_rect" : [ 1387.881897807121277, 740.371575593948364, 33.0, 22.0 ],
					"text" : "* 10."
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-419",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"patching_rect" : [ 1387.881897807121277, 707.038243532180786, 55.0, 22.0 ],
					"text" : "!/ 44100."
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-420",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1448.298562169075012, 748.704908609390259, 70.0, 22.0 ],
					"text" : "samples $1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-421",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"patching_rect" : [ 1667.048553824424744, 725.788242816925049, 30.0, 22.0 ],
					"text" : "/ 4."
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-422",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "signal" ],
					"patching_rect" : [ 1667.048553824424744, 748.704908609390259, 43.0, 22.0 ],
					"text" : "cycle~"
				}

			}
, 			{
				"box" : 				{
					"grid" : 2,
					"id" : "obj-423",
					"interval" : 51.0,
					"maxclass" : "live.scope~",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"patching_rect" : [ 1443.048553824424744, 834.000024855136871, 267.0, 139.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 29.729727745056152, 537.837801933288574, 352.559687756001949, 179.22077751159668 ],
					"samples" : 8266.166822867853625
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-424",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 1220.270188808441162, 849.999943256378174, 45.0, 22.0 ],
					"text" : "dac~ 4"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-425",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "signal" ],
					"patching_rect" : [ 1218.918837547302246, 732.43238353729248, 30.0, 22.0 ],
					"text" : "+~"
				}

			}
, 			{
				"box" : 				{
					"attr" : "range",
					"id" : "obj-426",
					"maxclass" : "attrui",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 1448.298562169075012, 717.454909801483154, 177.0, 22.0 ]
				}

			}
, 			{
				"box" : 				{
					"attr" : "interval",
					"id" : "obj-427",
					"maxclass" : "attrui",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 1448.298562169075012, 688.288244247436523, 150.0, 22.0 ]
				}

			}
, 			{
				"box" : 				{
					"attr" : "mode",
					"id" : "obj-428",
					"maxclass" : "attrui",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 1448.298562169075012, 659.121578693389893, 150.0, 22.0 ]
				}

			}
, 			{
				"box" : 				{
					"angle" : 270.0,
					"bgcolor" : [ 0.494117647058824, 0.780392156862745, 0.796078431372549, 1.0 ],
					"id" : "obj-429",
					"maxclass" : "panel",
					"mode" : 0,
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 1325.381900191307068, 621.621580123901367, 408.0, 319.0 ],
					"proportion" : 0.39
				}

			}
, 			{
				"box" : 				{
					"angle" : 270.0,
					"bgcolor" : [ 0.494117647058824, 0.780392156862745, 0.796078431372549, 1.0 ],
					"id" : "obj-430",
					"maxclass" : "panel",
					"mode" : 0,
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 1033.783714771270752, 617.567526340484619, 149.397595882415771, 238.55422568321228 ],
					"proportion" : 0.39
				}

			}
, 			{
				"box" : 				{
					"angle" : 270.0,
					"bgcolor" : [ 0.494117647058824, 0.780392156862745, 0.796078431372549, 1.0 ],
					"id" : "obj-431",
					"maxclass" : "panel",
					"mode" : 0,
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 1201.351271152496338, 621.621580123901367, 109.230779647827148, 272.307718276977539 ],
					"proportion" : 0.39
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-327",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1044.005219340324402, 305.962939143180847, 46.0, 22.0 ],
					"text" : "r funda"
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-328",
					"maxclass" : "dial",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1097.429872989654541, 275.825955033302307, 23.0, 23.0 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_longname" : "freqFineA/0",
							"parameter_mmax" : 1.0,
							"parameter_modmode" : 3,
							"parameter_shortname" : "dial",
							"parameter_type" : 0
						}

					}
,
					"size" : 1.0,
					"varname" : "dial[11]"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 12.0,
					"format" : 6,
					"id" : "obj-329",
					"maxclass" : "flonum",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 1097.429872989654541, 373.086221933364868, 51.0, 22.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 27.02702522277832, 845.945889472961426, 37.113399982452393, 22.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-330",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1072.772340536117554, 274.456092119216919, 24.0, 24.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 47.297294139862061, 827.026971817016602, 16.857143610715866, 16.857143610715866 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_enum" : [ "off", "on" ],
							"parameter_longname" : "incHarmA/0",
							"parameter_mmax" : 1,
							"parameter_modmode" : 0,
							"parameter_shortname" : "button[1]",
							"parameter_type" : 2
						}

					}
,
					"varname" : "button[18]"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-331",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1044.005219340324402, 274.456092119216919, 24.0, 24.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 27.02702522277832, 827.026971817016602, 16.857143610715866, 16.857143610715866 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_enum" : [ "off", "on" ],
							"parameter_longname" : "decHarmA/0",
							"parameter_mmax" : 1,
							"parameter_modmode" : 0,
							"parameter_shortname" : "button",
							"parameter_type" : 2
						}

					}
,
					"varname" : "button[19]"
				}

			}
, 			{
				"box" : 				{
					"format" : 6,
					"id" : "obj-332",
					"maxclass" : "flonum",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 1044.005219340324402, 373.086221933364868, 50.0, 22.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-333",
					"linecount" : 2,
					"maxclass" : "newobj",
					"numinlets" : 5,
					"numoutlets" : 3,
					"outlettype" : [ "signal", "float", "int" ],
					"patching_rect" : [ 1044.005219340324402, 334.730060338973999, 99.0, 35.0 ],
					"text" : "harmModTouch @harm 2"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-334",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1097.429872989654541, 305.962939143180847, 53.0, 22.0 ],
					"text" : "r amp0a"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-335",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1600.169562458992004, 460.757448434829712, 46.0, 22.0 ],
					"text" : "r funda"
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-336",
					"maxclass" : "dial",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1659.073667764663696, 426.510875582695007, 23.0, 23.0 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_longname" : "freqFineA/8",
							"parameter_mmax" : 1.0,
							"parameter_modmode" : 3,
							"parameter_shortname" : "dial",
							"parameter_type" : 0
						}

					}
,
					"size" : 1.0,
					"varname" : "dial[12]"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 12.0,
					"format" : 6,
					"id" : "obj-337",
					"maxclass" : "flonum",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 1659.073667764663696, 526.510868310928345, 51.0, 22.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 343.243220329284668, 845.945889472961426, 37.113399982452393, 22.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-338",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1628.936683654785156, 426.510875582695007, 24.0, 24.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 363.513489246368408, 827.026971817016602, 16.857143610715866, 16.857143610715866 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_enum" : [ "off", "on" ],
							"parameter_longname" : "incHarmA/8",
							"parameter_mmax" : 1,
							"parameter_modmode" : 0,
							"parameter_shortname" : "button[1]",
							"parameter_type" : 2
						}

					}
,
					"varname" : "button[20]"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-339",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1601.539425373077393, 426.510875582695007, 24.0, 24.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 343.243220329284668, 827.026971817016602, 16.857143610715866, 16.857143610715866 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_enum" : [ "off", "on" ],
							"parameter_longname" : "decHarmA/8",
							"parameter_mmax" : 1,
							"parameter_modmode" : 0,
							"parameter_shortname" : "button",
							"parameter_type" : 2
						}

					}
,
					"varname" : "button[21]"
				}

			}
, 			{
				"box" : 				{
					"format" : 6,
					"id" : "obj-340",
					"maxclass" : "flonum",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 1600.169562458992004, 526.510868310928345, 50.0, 22.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-341",
					"linecount" : 2,
					"maxclass" : "newobj",
					"numinlets" : 5,
					"numoutlets" : 3,
					"outlettype" : [ "signal", "float", "int" ],
					"patching_rect" : [ 1600.169562458992004, 488.154706716537476, 105.0, 35.0 ],
					"text" : "harmModTouch @harm 13"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-342",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1653.594216108322144, 460.757448434829712, 53.0, 22.0 ],
					"text" : "r amp8a"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-343",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1476.881900191307068, 460.757448434829712, 46.0, 22.0 ],
					"text" : "r funda"
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-344",
					"maxclass" : "dial",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1531.676416754722595, 426.510875582695007, 23.0, 23.0 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_longname" : "freqFineA/7",
							"parameter_mmax" : 1.0,
							"parameter_modmode" : 3,
							"parameter_shortname" : "dial",
							"parameter_type" : 0
						}

					}
,
					"size" : 1.0,
					"varname" : "dial[13]"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 12.0,
					"format" : 6,
					"id" : "obj-345",
					"maxclass" : "flonum",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 1528.936690926551819, 525.141005396842957, 51.0, 22.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 304.054033756256104, 845.945889472961426, 37.113399982452393, 22.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-346",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1505.64902138710022, 426.510875582695007, 24.0, 24.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 324.324302673339844, 827.026971817016602, 16.857143610715866, 16.857143610715866 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_enum" : [ "off", "on" ],
							"parameter_longname" : "incHarmA/7",
							"parameter_mmax" : 1,
							"parameter_modmode" : 0,
							"parameter_shortname" : "button[1]",
							"parameter_type" : 2
						}

					}
,
					"varname" : "button[22]"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-347",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1480.991488933563232, 426.510875582695007, 24.0, 24.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 304.054033756256104, 827.026971817016602, 16.857143610715866, 16.857143610715866 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_enum" : [ "off", "on" ],
							"parameter_longname" : "decHarmA/7",
							"parameter_mmax" : 1,
							"parameter_modmode" : 0,
							"parameter_shortname" : "button",
							"parameter_type" : 2
						}

					}
,
					"varname" : "button[23]"
				}

			}
, 			{
				"box" : 				{
					"format" : 6,
					"id" : "obj-348",
					"maxclass" : "flonum",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 1476.881900191307068, 525.141005396842957, 50.0, 22.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-349",
					"linecount" : 2,
					"maxclass" : "newobj",
					"numinlets" : 5,
					"numoutlets" : 3,
					"outlettype" : [ "signal", "float", "int" ],
					"patching_rect" : [ 1476.881900191307068, 488.154706716537476, 105.0, 35.0 ],
					"text" : "harmModTouch @harm 11"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-350",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1528.936690926551819, 460.757448434829712, 53.0, 22.0 ],
					"text" : "r amp7a"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-351",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1348.114786267280579, 460.757448434829712, 46.0, 22.0 ],
					"text" : "r funda"
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-352",
					"maxclass" : "dial",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1405.649028658866882, 426.510875582695007, 23.0, 23.0 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_longname" : "freqFineA/6",
							"parameter_mmax" : 1.0,
							"parameter_modmode" : 3,
							"parameter_shortname" : "dial",
							"parameter_type" : 0
						}

					}
,
					"size" : 1.0,
					"varname" : "dial[14]"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 12.0,
					"format" : 6,
					"id" : "obj-353",
					"maxclass" : "flonum",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 1405.649028658866882, 526.510868310928345, 51.0, 22.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 264.864847183227539, 845.945889472961426, 37.113399982452393, 22.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-354",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1378.251770377159119, 426.510875582695007, 24.0, 24.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 285.135116100311279, 827.026971817016602, 16.857143610715866, 16.857143610715866 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_enum" : [ "off", "on" ],
							"parameter_longname" : "incHarmA/6",
							"parameter_mmax" : 1,
							"parameter_modmode" : 0,
							"parameter_shortname" : "button[1]",
							"parameter_type" : 2
						}

					}
,
					"varname" : "button[24]"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-355",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1348.114786267280579, 426.510875582695007, 24.0, 24.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 264.864847183227539, 827.026971817016602, 16.857143610715866, 16.857143610715866 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_enum" : [ "off", "on" ],
							"parameter_longname" : "decHarmA/6",
							"parameter_mmax" : 1,
							"parameter_modmode" : 0,
							"parameter_shortname" : "button",
							"parameter_type" : 2
						}

					}
,
					"varname" : "button[25]"
				}

			}
, 			{
				"box" : 				{
					"format" : 6,
					"id" : "obj-356",
					"maxclass" : "flonum",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 1348.114786267280579, 526.510868310928345, 50.0, 22.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-357",
					"linecount" : 2,
					"maxclass" : "newobj",
					"numinlets" : 5,
					"numoutlets" : 3,
					"outlettype" : [ "signal", "float", "int" ],
					"patching_rect" : [ 1348.114786267280579, 488.154706716537476, 105.0, 35.0 ],
					"text" : "harmModTouch @harm 10"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-358",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1401.539439916610718, 460.757448434829712, 53.0, 22.0 ],
					"text" : "r amp6a"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-359",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1228.936712741851807, 463.497174263000488, 46.0, 22.0 ],
					"text" : "r funda"
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-360",
					"maxclass" : "dial",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1285.101092219352722, 429.250601410865784, 23.0, 23.0 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_longname" : "freqFineA/5",
							"parameter_mmax" : 1.0,
							"parameter_modmode" : 3,
							"parameter_shortname" : "dial",
							"parameter_type" : 0
						}

					}
,
					"size" : 1.0,
					"varname" : "dial[15]"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 12.0,
					"format" : 6,
					"id" : "obj-361",
					"maxclass" : "flonum",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 1285.101092219352722, 529.250594139099121, 51.0, 22.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 227.027011871337891, 845.945889472961426, 37.113399982452393, 22.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-362",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1259.073696851730347, 429.250601410865784, 24.0, 24.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 245.945929527282715, 827.026971817016602, 16.857143610715866, 16.857143610715866 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_enum" : [ "off", "on" ],
							"parameter_longname" : "incHarmA/5",
							"parameter_mmax" : 1,
							"parameter_modmode" : 0,
							"parameter_shortname" : "button[1]",
							"parameter_type" : 2
						}

					}
,
					"varname" : "button[26]"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-363",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1228.936712741851807, 429.250601410865784, 24.0, 24.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 224.324309349060059, 827.026971817016602, 16.857143610715866, 16.857143610715866 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_enum" : [ "off", "on" ],
							"parameter_longname" : "decHarmA/5",
							"parameter_mmax" : 1,
							"parameter_modmode" : 0,
							"parameter_shortname" : "button",
							"parameter_type" : 2
						}

					}
,
					"varname" : "button[27]"
				}

			}
, 			{
				"box" : 				{
					"format" : 6,
					"id" : "obj-364",
					"maxclass" : "flonum",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 1228.936712741851807, 529.250594139099121, 50.0, 22.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-365",
					"linecount" : 2,
					"maxclass" : "newobj",
					"numinlets" : 5,
					"numoutlets" : 3,
					"outlettype" : [ "signal", "float", "int" ],
					"patching_rect" : [ 1228.936712741851807, 488.154706716537476, 99.0, 35.0 ],
					"text" : "harmModTouch @harm 9"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-366",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1280.991503477096558, 463.497174263000488, 53.0, 22.0 ],
					"text" : "r amp5a"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-367",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1600.169562458992004, 310.072527885437012, 46.0, 22.0 ],
					"text" : "r funda"
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-368",
					"maxclass" : "dial",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1659.073667764663696, 275.825955033302307, 23.0, 23.0 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_longname" : "freqFineA/4",
							"parameter_mmax" : 1.0,
							"parameter_modmode" : 3,
							"parameter_shortname" : "dial",
							"parameter_type" : 0
						}

					}
,
					"size" : 1.0,
					"varname" : "dial[16]"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 12.0,
					"format" : 6,
					"id" : "obj-369",
					"maxclass" : "flonum",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 1656.33394193649292, 375.825947761535645, 51.0, 22.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 185.135122776031494, 845.945889472961426, 37.113399982452393, 22.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-370",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1628.936683654785156, 274.456092119216919, 24.0, 24.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 204.054040431976318, 827.026971817016602, 16.857143610715866, 16.857143610715866 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_enum" : [ "off", "on" ],
							"parameter_longname" : "incHarmA/4",
							"parameter_mmax" : 1,
							"parameter_modmode" : 0,
							"parameter_shortname" : "button[1]",
							"parameter_type" : 2
						}

					}
,
					"varname" : "button[28]"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-371",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1600.169562458992004, 274.456092119216919, 24.0, 24.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 185.135122776031494, 827.026971817016602, 16.857143610715866, 16.857143610715866 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_enum" : [ "off", "on" ],
							"parameter_longname" : "decHarmA/4",
							"parameter_mmax" : 1,
							"parameter_modmode" : 0,
							"parameter_shortname" : "button",
							"parameter_type" : 2
						}

					}
,
					"varname" : "button[29]"
				}

			}
, 			{
				"box" : 				{
					"format" : 6,
					"id" : "obj-372",
					"maxclass" : "flonum",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 1600.169562458992004, 375.825947761535645, 50.0, 22.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-373",
					"linecount" : 2,
					"maxclass" : "newobj",
					"numinlets" : 5,
					"numoutlets" : 3,
					"outlettype" : [ "signal", "float", "int" ],
					"patching_rect" : [ 1600.169562458992004, 338.839649081230164, 99.0, 35.0 ],
					"text" : "harmModTouch @harm 8"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-374",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1653.594216108322144, 310.072527885437012, 53.0, 22.0 ],
					"text" : "r amp4a"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-375",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1476.881900191307068, 308.702664971351624, 46.0, 22.0 ],
					"text" : "r funda"
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-376",
					"maxclass" : "dial",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1531.676416754722595, 275.825955033302307, 23.0, 23.0 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_longname" : "freqFineA/3",
							"parameter_mmax" : 1.0,
							"parameter_modmode" : 3,
							"parameter_shortname" : "dial",
							"parameter_type" : 0
						}

					}
,
					"size" : 1.0,
					"varname" : "dial[17]"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 12.0,
					"format" : 6,
					"id" : "obj-377",
					"maxclass" : "flonum",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 1528.936690926551819, 373.086221933364868, 51.0, 22.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 145.94593620300293, 845.945889472961426, 37.113399982452393, 22.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-378",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1504.279158473014832, 274.456092119216919, 24.0, 24.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 166.21620512008667, 827.026971817016602, 16.857143610715866, 16.857143610715866 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_enum" : [ "off", "on" ],
							"parameter_longname" : "incHarmA/3",
							"parameter_mmax" : 1,
							"parameter_modmode" : 0,
							"parameter_shortname" : "button[1]",
							"parameter_type" : 2
						}

					}
,
					"varname" : "button[30]"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-379",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1476.881900191307068, 274.456092119216919, 24.0, 24.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 145.94593620300293, 827.026971817016602, 16.857143610715866, 16.857143610715866 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_enum" : [ "off", "on" ],
							"parameter_longname" : "decHarmA/3",
							"parameter_mmax" : 1,
							"parameter_modmode" : 0,
							"parameter_shortname" : "button",
							"parameter_type" : 2
						}

					}
,
					"varname" : "button[31]"
				}

			}
, 			{
				"box" : 				{
					"format" : 6,
					"id" : "obj-380",
					"maxclass" : "flonum",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 1476.881900191307068, 373.086221933364868, 50.0, 22.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-381",
					"linecount" : 2,
					"maxclass" : "newobj",
					"numinlets" : 5,
					"numoutlets" : 3,
					"outlettype" : [ "signal", "float", "int" ],
					"patching_rect" : [ 1476.881900191307068, 336.099923253059387, 99.0, 35.0 ],
					"text" : "harmModTouch @harm 7"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-382",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1524.827102184295654, 308.702664971351624, 53.0, 22.0 ],
					"text" : "r amp3a"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-383",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1348.114786267280579, 308.702664971351624, 46.0, 22.0 ],
					"text" : "r funda"
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-384",
					"maxclass" : "dial",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1404.279165744781494, 275.825955033302307, 23.0, 23.0 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_longname" : "freqFineA/2",
							"parameter_mmax" : 1.0,
							"parameter_modmode" : 3,
							"parameter_shortname" : "dial",
							"parameter_type" : 0
						}

					}
,
					"size" : 1.0,
					"varname" : "dial[18]"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 12.0,
					"format" : 6,
					"id" : "obj-385",
					"maxclass" : "flonum",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 1405.649028658866882, 375.825947761535645, 51.0, 22.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 105.405398368835449, 845.945889472961426, 37.113399982452393, 22.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-386",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1376.88190746307373, 274.456092119216919, 24.0, 24.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 127.027018547058105, 827.026971817016602, 16.857143610715866, 16.857143610715866 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_enum" : [ "off", "on" ],
							"parameter_longname" : "incHarmA/2",
							"parameter_mmax" : 1,
							"parameter_modmode" : 0,
							"parameter_shortname" : "button[1]",
							"parameter_type" : 2
						}

					}
,
					"varname" : "button[32]"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-387",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1348.114786267280579, 274.456092119216919, 24.0, 24.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 105.405398368835449, 827.026971817016602, 16.857143610715866, 16.857143610715866 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_enum" : [ "off", "on" ],
							"parameter_longname" : "decHarmA/2",
							"parameter_mmax" : 1,
							"parameter_modmode" : 0,
							"parameter_shortname" : "button",
							"parameter_type" : 2
						}

					}
,
					"varname" : "button[33]"
				}

			}
, 			{
				"box" : 				{
					"format" : 6,
					"id" : "obj-388",
					"maxclass" : "flonum",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 1348.114786267280579, 375.825947761535645, 50.0, 22.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-389",
					"linecount" : 2,
					"maxclass" : "newobj",
					"numinlets" : 5,
					"numoutlets" : 3,
					"outlettype" : [ "signal", "float", "int" ],
					"patching_rect" : [ 1348.114786267280579, 336.099923253059387, 99.0, 35.0 ],
					"text" : "harmModTouch @harm 5"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-390",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1400.16957700252533, 308.702664971351624, 53.0, 22.0 ],
					"text" : "r amp2a"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-391",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1228.936712741851807, 308.702664971351624, 46.0, 22.0 ],
					"text" : "r funda"
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-392",
					"maxclass" : "dial",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1285.101092219352722, 275.825955033302307, 23.0, 23.0 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_longname" : "freqFineA/1",
							"parameter_mmax" : 1.0,
							"parameter_modmode" : 3,
							"parameter_shortname" : "dial",
							"parameter_type" : 0
						}

					}
,
					"size" : 1.0,
					"varname" : "dial[19]"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 12.0,
					"format" : 6,
					"id" : "obj-393",
					"maxclass" : "flonum",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 1278.251777648925781, 374.456084847450256, 51.0, 22.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 66.216211795806885, 845.945889472961426, 37.113399982452393, 22.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-394",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1259.073696851730347, 274.456092119216919, 24.0, 24.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 85.135129451751709, 827.026971817016602, 16.857143610715866, 16.857143610715866 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_enum" : [ "off", "on" ],
							"parameter_longname" : "incHarmA/1",
							"parameter_mmax" : 1,
							"parameter_modmode" : 0,
							"parameter_shortname" : "button[1]",
							"parameter_type" : 2
						}

					}
,
					"varname" : "button[34]"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-395",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1228.936712741851807, 274.456092119216919, 24.0, 24.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 66.216211795806885, 827.026971817016602, 16.857143610715866, 16.857143610715866 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_enum" : [ "off", "on" ],
							"parameter_longname" : "decHarmA/1",
							"parameter_mmax" : 1,
							"parameter_modmode" : 0,
							"parameter_shortname" : "button",
							"parameter_type" : 2
						}

					}
,
					"varname" : "button[35]"
				}

			}
, 			{
				"box" : 				{
					"format" : 6,
					"id" : "obj-396",
					"maxclass" : "flonum",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 1226.936712741851807, 374.456084847450256, 50.0, 22.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-397",
					"linecount" : 2,
					"maxclass" : "newobj",
					"numinlets" : 5,
					"numoutlets" : 3,
					"outlettype" : [ "signal", "float", "int" ],
					"patching_rect" : [ 1228.936712741851807, 336.099923253059387, 99.0, 35.0 ],
					"text" : "harmModTouch @harm 3"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-398",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1280.991503477096558, 308.702664971351624, 53.0, 22.0 ],
					"text" : "r amp1a"
				}

			}
, 			{
				"box" : 				{
					"angle" : 270.0,
					"bgcolor" : [ 0.494117647058824, 0.780392156862745, 0.796078431372549, 1.0 ],
					"id" : "obj-399",
					"maxclass" : "panel",
					"mode" : 0,
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 1031.676453113555908, 247.058833837509155, 837.647093772888184, 351.764720559120178 ],
					"proportion" : 0.39
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-264",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 1772.340412855148315, 25.531914710998535, 10.0, 10.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-265",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 1691.489349603652954, 25.531914710998535, 10.0, 10.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-266",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 1610.638286352157593, 25.531914710998535, 10.0, 10.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-267",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 1534.042542219161987, 25.531914710998535, 10.0, 10.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-268",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 1457.446798086166382, 25.531914710998535, 10.0, 10.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-269",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 1376.595734834671021, 25.531914710998535, 10.0, 10.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-270",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 1295.744671583175659, 25.531914710998535, 10.0, 10.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-271",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 1221.276587009429932, 25.531914710998535, 10.0, 10.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-272",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 1140.42552375793457, 21.276595592498779, 10.0, 10.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-273",
					"maxclass" : "meter~",
					"numinlets" : 1,
					"numleds" : 10,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"patching_rect" : [ 1785.106370210647583, 38.297872066497803, 19.0, 62.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 368.918894290924072, 722.972924709320068, 12.105263590812683, 98.947371959686279 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-274",
					"maxclass" : "meter~",
					"numinlets" : 1,
					"numleds" : 10,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"patching_rect" : [ 1706.3829665184021, 40.425531625747681, 19.0, 62.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 328.378356456756592, 722.972924709320068, 12.105263590812683, 98.947371959686279 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-275",
					"maxclass" : "meter~",
					"numinlets" : 1,
					"numleds" : 10,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"patching_rect" : [ 1631.914881944656372, 40.425531625747681, 19.0, 62.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 289.189169883728027, 722.972924709320068, 12.105263590812683, 98.947371959686279 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-276",
					"maxclass" : "meter~",
					"numinlets" : 1,
					"numleds" : 10,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"patching_rect" : [ 1551.063818693161011, 40.425531625747681, 19.0, 62.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 249.999983310699463, 722.972924709320068, 12.105263590812683, 98.947371959686279 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-277",
					"maxclass" : "meter~",
					"numinlets" : 1,
					"numleds" : 10,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"patching_rect" : [ 1472.340415000915527, 40.425531625747681, 19.0, 62.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 210.810796737670898, 722.972924709320068, 12.105263590812683, 98.947371959686279 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-278",
					"maxclass" : "meter~",
					"numinlets" : 1,
					"numleds" : 10,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"patching_rect" : [ 1391.489351749420166, 40.425531625747681, 19.0, 62.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 170.270258903503418, 722.972924709320068, 12.105263590812683, 98.947371959686279 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-279",
					"maxclass" : "meter~",
					"numinlets" : 1,
					"numleds" : 10,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"patching_rect" : [ 1310.638288497924805, 40.425531625747681, 19.0, 62.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 131.081072330474854, 722.972924709320068, 12.105263590812683, 98.947371959686279 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-280",
					"maxclass" : "meter~",
					"numinlets" : 1,
					"numleds" : 10,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"patching_rect" : [ 1234.042544364929199, 40.425531625747681, 19.0, 62.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 91.891885757446289, 722.972924709320068, 12.105263590812683, 98.947371959686279 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-281",
					"maxclass" : "meter~",
					"numinlets" : 1,
					"numleds" : 10,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"patching_rect" : [ 1157.446800231933594, 40.425531625747681, 19.0, 62.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 52.702699184417725, 722.972924709320068, 12.105263590812683, 98.947371959686279 ]
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-282",
					"maxclass" : "slider",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1772.340412855148315, 40.425531625747681, 11.0, 61.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 345.9459228515625, 722.972924709320068, 22.333333998918533, 99.333336293697357 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_longname" : "harmAmpA/8",
							"parameter_mmax" : 1.0,
							"parameter_modmode" : 3,
							"parameter_shortname" : "slider[9]",
							"parameter_type" : 0
						}

					}
,
					"size" : 1.0,
					"varname" : "slider[11]"
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-283",
					"maxclass" : "slider",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1691.489349603652954, 40.425531625747681, 11.0, 61.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 305.40538501739502, 722.972924709320068, 22.333333998918533, 99.333336293697357 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_longname" : "harmAmpA/7",
							"parameter_mmax" : 1.0,
							"parameter_modmode" : 3,
							"parameter_shortname" : "slider[8]",
							"parameter_type" : 0
						}

					}
,
					"size" : 1.0,
					"varname" : "slider[12]"
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-284",
					"maxclass" : "slider",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1610.638286352157593, 40.425531625747681, 11.0, 61.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 266.216198444366455, 722.972924709320068, 22.333333998918533, 99.333336293697357 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_longname" : "harmAmpA/6",
							"parameter_mmax" : 1.0,
							"parameter_modmode" : 3,
							"parameter_shortname" : "slider[7]",
							"parameter_type" : 0
						}

					}
,
					"size" : 1.0,
					"varname" : "slider[13]"
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-285",
					"maxclass" : "slider",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1534.042542219161987, 40.425531625747681, 11.0, 61.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 225.675660610198975, 722.972924709320068, 22.333333998918533, 99.333336293697357 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_longname" : "harmAmpA/5",
							"parameter_mmax" : 1.0,
							"parameter_modmode" : 3,
							"parameter_shortname" : "slider[6]",
							"parameter_type" : 0
						}

					}
,
					"size" : 1.0,
					"varname" : "slider[14]"
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-286",
					"maxclass" : "slider",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1457.446798086166382, 40.425531625747681, 11.0, 61.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 187.837825298309326, 722.972924709320068, 22.333333998918533, 99.333336293697357 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_longname" : "harmAmpA/4",
							"parameter_mmax" : 1.0,
							"parameter_modmode" : 3,
							"parameter_shortname" : "slider[5]",
							"parameter_type" : 0
						}

					}
,
					"size" : 1.0,
					"varname" : "slider[15]"
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-287",
					"maxclass" : "slider",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1376.595734834671021, 40.425531625747681, 11.0, 61.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 147.297287464141846, 722.972924709320068, 22.333333998918533, 99.333336293697357 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_longname" : "harmAmpA/3",
							"parameter_mmax" : 1.0,
							"parameter_modmode" : 3,
							"parameter_shortname" : "slider[4]",
							"parameter_type" : 0
						}

					}
,
					"size" : 1.0,
					"varname" : "slider[16]"
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-288",
					"maxclass" : "slider",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1295.744671583175659, 40.425531625747681, 11.0, 61.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 108.108100891113281, 722.972924709320068, 22.333333998918533, 99.333336293697357 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_longname" : "harmAmpA/2",
							"parameter_mmax" : 1.0,
							"parameter_modmode" : 3,
							"parameter_shortname" : "slider[3]",
							"parameter_type" : 0
						}

					}
,
					"size" : 1.0,
					"varname" : "slider[17]"
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-289",
					"maxclass" : "slider",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1221.276587009429932, 40.425531625747681, 11.0, 61.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 68.918914318084717, 722.972924709320068, 22.333333998918533, 99.333336293697357 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_longname" : "harmAmpA/1",
							"parameter_mmax" : 1.0,
							"parameter_modmode" : 3,
							"parameter_shortname" : "slider[2]",
							"parameter_type" : 0
						}

					}
,
					"size" : 1.0,
					"varname" : "slider[18]"
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-290",
					"maxclass" : "slider",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1140.42552375793457, 40.425531625747681, 11.0, 61.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 29.729727745056152, 722.972924709320068, 22.333333998918533, 99.333336293697357 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_longname" : "harmAmpA/0",
							"parameter_mmax" : 1.0,
							"parameter_modmode" : 3,
							"parameter_shortname" : "slider[1]",
							"parameter_type" : 0
						}

					}
,
					"size" : 1.0,
					"varname" : "slider[19]"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-291",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1770.212753295898438, 134.04255223274231, 51.0, 22.0 ],
					"text" : "r seqOn"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-292",
					"maxclass" : "newobj",
					"numinlets" : 3,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1770.212753295898438, 163.829786062240601, 51.0, 22.0 ],
					"text" : "switch"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-293",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1691.489349603652954, 134.04255223274231, 51.0, 22.0 ],
					"text" : "r seqOn"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-294",
					"maxclass" : "newobj",
					"numinlets" : 3,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1691.489349603652954, 163.829786062240601, 51.0, 22.0 ],
					"text" : "switch"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-295",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1612.765945911407471, 134.04255223274231, 51.0, 22.0 ],
					"text" : "r seqOn"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-296",
					"maxclass" : "newobj",
					"numinlets" : 3,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1612.765945911407471, 163.829786062240601, 51.0, 22.0 ],
					"text" : "switch"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-297",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1534.042542219161987, 134.04255223274231, 51.0, 22.0 ],
					"text" : "r seqOn"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-298",
					"maxclass" : "newobj",
					"numinlets" : 3,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1534.042542219161987, 163.829786062240601, 51.0, 22.0 ],
					"text" : "switch"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-299",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1457.446798086166382, 134.04255223274231, 51.0, 22.0 ],
					"text" : "r seqOn"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-300",
					"maxclass" : "newobj",
					"numinlets" : 3,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1457.446798086166382, 163.829786062240601, 51.0, 22.0 ],
					"text" : "switch"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-301",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1376.595734834671021, 134.04255223274231, 51.0, 22.0 ],
					"text" : "r seqOn"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-302",
					"maxclass" : "newobj",
					"numinlets" : 3,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1376.595734834671021, 163.829786062240601, 51.0, 22.0 ],
					"text" : "switch"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-303",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1295.744671583175659, 134.04255223274231, 51.0, 22.0 ],
					"text" : "r seqOn"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-304",
					"maxclass" : "newobj",
					"numinlets" : 3,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1295.744671583175659, 163.829786062240601, 51.0, 22.0 ],
					"text" : "switch"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-305",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1221.276587009429932, 134.04255223274231, 51.0, 22.0 ],
					"text" : "r seqOn"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-306",
					"maxclass" : "newobj",
					"numinlets" : 3,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1221.276587009429932, 163.829786062240601, 51.0, 22.0 ],
					"text" : "switch"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-307",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1772.340412855148315, 110.638297080993652, 39.0, 22.0 ],
					"text" : "r st8a"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-308",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1691.489349603652954, 110.638297080993652, 39.0, 22.0 ],
					"text" : "r st7a"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-309",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1612.765945911407471, 110.638297080993652, 39.0, 22.0 ],
					"text" : "r st6a"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-310",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1534.042542219161987, 110.638297080993652, 39.0, 22.0 ],
					"text" : "r st5a"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-311",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1457.446798086166382, 110.638297080993652, 39.0, 22.0 ],
					"text" : "r st4a"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-312",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1376.595734834671021, 110.638297080993652, 39.0, 22.0 ],
					"text" : "r st3a"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-313",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1295.744671583175659, 110.638297080993652, 39.0, 22.0 ],
					"text" : "r st2a"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-314",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1221.276587009429932, 110.638297080993652, 39.0, 22.0 ],
					"text" : "r st1a"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-315",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1140.42552375793457, 110.638297080993652, 39.0, 22.0 ],
					"text" : "r st0a"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-316",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1140.42552375793457, 136.170211791992188, 51.0, 22.0 ],
					"text" : "r seqOn"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-317",
					"maxclass" : "newobj",
					"numinlets" : 3,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1140.42552375793457, 163.829786062240601, 51.0, 22.0 ],
					"text" : "switch"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-318",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 1770.212753295898438, 187.234041213989258, 55.0, 22.0 ],
					"text" : "s amp8a"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-319",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 1691.489349603652954, 187.234041213989258, 55.0, 22.0 ],
					"text" : "s amp7a"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-320",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 1612.765945911407471, 187.234041213989258, 55.0, 22.0 ],
					"text" : "s amp6a"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-321",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 1534.042542219161987, 187.234041213989258, 55.0, 22.0 ],
					"text" : "s amp5a"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-322",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 1457.446798086166382, 187.234041213989258, 55.0, 22.0 ],
					"text" : "s amp4a"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-323",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 1376.595734834671021, 187.234041213989258, 55.0, 22.0 ],
					"text" : "s amp3a"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-324",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 1295.744671583175659, 187.234041213989258, 55.0, 22.0 ],
					"text" : "s amp2a"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-325",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 1221.276587009429932, 187.234041213989258, 55.0, 22.0 ],
					"text" : "s amp1a"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-326",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 1140.42552375793457, 189.361700773239136, 55.0, 22.0 ],
					"text" : "s amp0a"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-25",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"patching_rect" : [ 39.717391014099121, 972.710446119308472, 58.0, 22.0 ],
					"text" : "loadbang"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-7",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 30.346160888671875, 438.0, 107.0, 22.0 ],
					"text" : "prepend /fundFreq"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-232",
					"maxclass" : "newobj",
					"numinlets" : 9,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patcher" : 					{
						"fileversion" : 1,
						"appversion" : 						{
							"major" : 9,
							"minor" : 0,
							"revision" : 7,
							"architecture" : "x64",
							"modernui" : 1
						}
,
						"classnamespace" : "box",
						"rect" : [ 436.0, 183.0, 1000.0, 739.0 ],
						"gridsize" : [ 15.0, 15.0 ],
						"boxes" : [ 							{
								"box" : 								{
									"id" : "obj-17",
									"linecount" : 2,
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 724.742227435112, 98.400001466274261, 77.319583296775818, 35.0 ],
									"text" : "prepend /harmLabel8"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-16",
									"linecount" : 2,
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 641.237077474594116, 98.400001466274261, 77.319583296775818, 35.0 ],
									"text" : "prepend /harmLabel7"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-15",
									"linecount" : 2,
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 556.700999736785889, 98.400001466274261, 77.319583296775818, 35.0 ],
									"text" : "prepend /harmLabel6"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-14",
									"linecount" : 2,
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 472.164921998977661, 98.400001466274261, 77.319583296775818, 35.0 ],
									"text" : "prepend /harmLabel5"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-13",
									"linecount" : 2,
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 387.628844261169434, 98.400001466274261, 77.319583296775818, 35.0 ],
									"text" : "prepend /harmLabel4"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-12",
									"linecount" : 2,
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 303.092766523361206, 98.400001466274261, 77.319583296775818, 35.0 ],
									"text" : "prepend /harmLabel3"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-11",
									"linecount" : 2,
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 218.556688785552979, 98.400001466274261, 77.319583296775818, 35.0 ],
									"text" : "prepend /harmLabel2"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-10",
									"linecount" : 2,
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 134.020611047744751, 98.400001466274261, 77.319583296775818, 35.0 ],
									"text" : "prepend /harmLabel1"
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-9",
									"index" : 9,
									"maxclass" : "inlet",
									"numinlets" : 0,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 723.711299657821655, 57.731955528259277, 23.0, 23.0 ]
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-8",
									"index" : 8,
									"maxclass" : "inlet",
									"numinlets" : 0,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 639.497387200593948, 58.0, 23.0, 23.0 ]
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-7",
									"index" : 7,
									"maxclass" : "inlet",
									"numinlets" : 0,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 555.283474743366241, 58.0, 23.0, 23.0 ]
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-6",
									"index" : 6,
									"maxclass" : "inlet",
									"numinlets" : 0,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 471.069562286138535, 58.0, 23.0, 23.0 ]
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-5",
									"index" : 5,
									"maxclass" : "inlet",
									"numinlets" : 0,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 386.855649828910828, 58.0, 23.0, 23.0 ]
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-4",
									"index" : 4,
									"maxclass" : "inlet",
									"numinlets" : 0,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 302.641737371683121, 58.0, 23.0, 23.0 ]
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-3",
									"index" : 3,
									"maxclass" : "inlet",
									"numinlets" : 0,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 218.427824914455414, 58.0, 23.0, 23.0 ]
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-2",
									"index" : 2,
									"maxclass" : "inlet",
									"numinlets" : 0,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 134.213912457227707, 58.0, 23.0, 23.0 ]
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-1",
									"index" : 1,
									"maxclass" : "inlet",
									"numinlets" : 0,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 50.0, 58.0, 23.0, 23.0 ]
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-227",
									"linecount" : 2,
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 50.0, 98.400001466274261, 78.0, 35.0 ],
									"text" : "prepend /harmLabel0"
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-231",
									"index" : 1,
									"maxclass" : "outlet",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 50.0, 210.0, 30.0, 30.0 ]
								}

							}
 ],
						"lines" : [ 							{
								"patchline" : 								{
									"destination" : [ "obj-227", 0 ],
									"source" : [ "obj-1", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-231", 0 ],
									"source" : [ "obj-10", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-231", 0 ],
									"source" : [ "obj-11", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-231", 0 ],
									"source" : [ "obj-12", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-231", 0 ],
									"source" : [ "obj-13", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-231", 0 ],
									"source" : [ "obj-14", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-231", 0 ],
									"source" : [ "obj-15", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-231", 0 ],
									"source" : [ "obj-16", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-231", 0 ],
									"source" : [ "obj-17", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-10", 0 ],
									"source" : [ "obj-2", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-231", 0 ],
									"source" : [ "obj-227", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-11", 0 ],
									"source" : [ "obj-3", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-12", 0 ],
									"source" : [ "obj-4", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-13", 0 ],
									"source" : [ "obj-5", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-14", 0 ],
									"source" : [ "obj-6", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-15", 0 ],
									"source" : [ "obj-7", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-16", 0 ],
									"source" : [ "obj-8", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-17", 0 ],
									"source" : [ "obj-9", 0 ]
								}

							}
 ]
					}
,
					"patching_rect" : [ 440.506323337554932, 563.291131854057312, 186.714287281036377, 22.0 ],
					"text" : "p oscSend"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-228",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 231.645566582679749, 563.291131854057312, 165.0, 22.0 ],
					"text" : "udpsend 172.20.10.13 30337"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-217",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 215.804344177246094, 1020.536532163619995, 30.0, 22.0 ],
					"text" : "1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-211",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 183.195649147033691, 1020.536532163619995, 30.0, 22.0 ],
					"text" : "0"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-172",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 30.171716332435608, 271.717158436775208, 24.0, 24.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 128.543745145201683, 399.520818054676056, 24.0, 24.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-168",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 720.0, 25.0, 10.0, 10.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-169",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 639.0, 25.0, 10.0, 10.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-170",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 559.0, 25.0, 10.0, 10.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-130",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 481.0, 25.0, 10.0, 10.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-132",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 403.0, 25.0, 10.0, 10.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-167",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 324.0, 25.0, 10.0, 10.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-129",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 244.0, 25.0, 10.0, 10.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-128",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 167.0, 25.0, 10.0, 10.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-89",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 87.0, 22.0, 10.0, 10.0 ]
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-82",
					"maxclass" : "slider",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 32.0, 41.0, 11.0, 61.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 95.348838686943054, 361.240315675735474, 27.90697717666626, 61.240311026573181 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_longname" : "ampFund",
							"parameter_mmax" : 1.0,
							"parameter_modmode" : 3,
							"parameter_shortname" : "slider[1]",
							"parameter_type" : 0
						}

					}
,
					"size" : 1.0,
					"varname" : "slider[10]"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-81",
					"maxclass" : "meter~",
					"numinlets" : 1,
					"numleds" : 10,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"patching_rect" : [ 733.0, 39.0, 19.0, 62.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 354.696514099836349, 202.192988187074661, 12.105263590812683, 98.947371959686279 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-80",
					"maxclass" : "meter~",
					"numinlets" : 1,
					"numleds" : 10,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"patching_rect" : [ 654.0, 41.0, 19.0, 62.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 314.210537552833557, 202.192988187074661, 12.105263590812683, 98.947371959686279 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-79",
					"maxclass" : "meter~",
					"numinlets" : 1,
					"numleds" : 10,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"patching_rect" : [ 577.0, 41.0, 19.0, 62.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 274.736851930618286, 202.105270385742188, 12.105263590812683, 98.947371959686279 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-68",
					"maxclass" : "meter~",
					"numinlets" : 1,
					"numleds" : 10,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"patching_rect" : [ 497.0, 41.0, 19.0, 62.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 235.263166308403015, 202.105270385742188, 12.105263590812683, 98.947371959686279 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-54",
					"maxclass" : "meter~",
					"numinlets" : 1,
					"numleds" : 10,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"patching_rect" : [ 420.0, 41.0, 19.0, 62.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 195.789480686187744, 202.105270385742188, 12.105263590812683, 98.947371959686279 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-53",
					"maxclass" : "meter~",
					"numinlets" : 1,
					"numleds" : 10,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"patching_rect" : [ 339.0, 41.0, 19.0, 62.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 156.315795063972473, 202.105270385742188, 12.105263590812683, 98.947371959686279 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-45",
					"maxclass" : "meter~",
					"numinlets" : 1,
					"numleds" : 10,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"patching_rect" : [ 259.0, 41.0, 19.0, 62.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 116.842109441757202, 202.105270385742188, 12.105263590812683, 98.947371959686279 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-38",
					"maxclass" : "meter~",
					"numinlets" : 1,
					"numleds" : 10,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"patching_rect" : [ 181.0, 41.0, 19.0, 62.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 77.368423819541931, 202.105270385742188, 12.105263590812683, 98.947371959686279 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-13",
					"maxclass" : "meter~",
					"numinlets" : 1,
					"numleds" : 10,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"patching_rect" : [ 103.0, 41.0, 19.0, 62.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 37.89473819732666, 202.105270385742188, 12.105263590812683, 98.947371959686279 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-35",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 294.065212249755859, 1231.40609335899353, 35.0, 22.0 ],
					"text" : "clear"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-75",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 207.108692169189453, 1203.145224332809448, 47.0, 22.0 ],
					"text" : "gridList"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-78",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 207.108692169189453, 1177.058268308639526, 104.0, 22.0 ],
					"text" : "udpreceive 30338"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-88",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 176.814815998077393, 662.963015913963318, 39.0, 22.0 ],
					"text" : "r fund"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-87",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 87.543477058410645, 1016.188706159591675, 65.0, 22.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 418.524133920669556, 394.753149449825287, 65.0, 22.0 ],
					"text" : "sequencer"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-77",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 39.717391014099121, 1016.188706159591675, 42.0, 22.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 441.712539911270142, 367.941555023193359, 42.0, 22.0 ],
					"text" : "faders"
				}

			}
, 			{
				"box" : 				{
					"bgcolor" : [ 0.125490196078431, 0.125490196078431, 0.125490196078431, 1.0 ],
					"elementcolor" : [ 0.717647058823529, 0.831372549019608, 0.572549019607843, 1.0 ],
					"floatoutput" : 1,
					"id" : "obj-70",
					"maxclass" : "slider",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 73.846160888671875, 508.692355155944824, 20.0, 140.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 382.94574236869812, 14.271288514137268, 53.246752738952637, 329.792204856872559 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_longname" : "mainAmp",
							"parameter_mmax" : 1.0,
							"parameter_modmode" : 3,
							"parameter_shortname" : "slider",
							"parameter_type" : 0
						}

					}
,
					"size" : 1.0,
					"varname" : "slider"
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-67",
					"maxclass" : "slider",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 720.0, 41.0, 11.0, 61.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 330.66667652130127, 202.000006020069122, 22.333333998918533, 99.333336293697357 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_longname" : "harmAmp/8",
							"parameter_mmax" : 1.0,
							"parameter_modmode" : 3,
							"parameter_shortname" : "slider[9]",
							"parameter_type" : 0
						}

					}
,
					"size" : 1.0,
					"varname" : "slider[9]"
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-56",
					"maxclass" : "slider",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 639.0, 41.0, 11.0, 61.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 290.666675329208374, 202.000006020069122, 22.333333998918533, 99.333336293697357 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_longname" : "harmAmp/7",
							"parameter_mmax" : 1.0,
							"parameter_modmode" : 3,
							"parameter_shortname" : "slider[8]",
							"parameter_type" : 0
						}

					}
,
					"size" : 1.0,
					"varname" : "slider[8]"
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-52",
					"maxclass" : "slider",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 559.0, 41.0, 11.0, 61.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 251.33334082365036, 202.000006020069122, 22.333333998918533, 99.333336293697357 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_longname" : "harmAmp/6",
							"parameter_mmax" : 1.0,
							"parameter_modmode" : 3,
							"parameter_shortname" : "slider[7]",
							"parameter_type" : 0
						}

					}
,
					"size" : 1.0,
					"varname" : "slider[7]"
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-51",
					"maxclass" : "slider",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 481.0, 41.0, 11.0, 61.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 211.333339631557465, 202.000006020069122, 22.333333998918533, 99.333336293697357 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_longname" : "harmAmp/5",
							"parameter_mmax" : 1.0,
							"parameter_modmode" : 3,
							"parameter_shortname" : "slider[6]",
							"parameter_type" : 0
						}

					}
,
					"size" : 1.0,
					"varname" : "slider[6]"
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-50",
					"maxclass" : "slider",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 403.0, 41.0, 11.0, 61.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 172.666671812534332, 202.000006020069122, 22.333333998918533, 99.333336293697357 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_longname" : "harmAmp/4",
							"parameter_mmax" : 1.0,
							"parameter_modmode" : 3,
							"parameter_shortname" : "slider[5]",
							"parameter_type" : 0
						}

					}
,
					"size" : 1.0,
					"varname" : "slider[5]"
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-41",
					"maxclass" : "slider",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 324.0, 41.0, 11.0, 61.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 133.333337306976318, 202.000006020069122, 22.333333998918533, 99.333336293697357 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_longname" : "harmAmp/3",
							"parameter_mmax" : 1.0,
							"parameter_modmode" : 3,
							"parameter_shortname" : "slider[4]",
							"parameter_type" : 0
						}

					}
,
					"size" : 1.0,
					"varname" : "slider[4]"
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-40",
					"maxclass" : "slider",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 244.0, 41.0, 11.0, 61.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 93.333336114883423, 202.000006020069122, 22.333333998918533, 99.333336293697357 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_longname" : "harmAmp/2",
							"parameter_mmax" : 1.0,
							"parameter_modmode" : 3,
							"parameter_shortname" : "slider[3]",
							"parameter_type" : 0
						}

					}
,
					"size" : 1.0,
					"varname" : "slider[3]"
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-8",
					"maxclass" : "slider",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 167.0, 41.0, 11.0, 61.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 54.000001609325409, 202.000006020069122, 22.333333998918533, 99.333336293697357 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_longname" : "harmAmp/1",
							"parameter_mmax" : 1.0,
							"parameter_modmode" : 3,
							"parameter_shortname" : "slider[2]",
							"parameter_type" : 0
						}

					}
,
					"size" : 1.0,
					"varname" : "slider[2]"
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-3",
					"maxclass" : "slider",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 87.0, 41.0, 11.0, 61.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 14.666667103767395, 202.000006020069122, 22.333333998918533, 99.333336293697357 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_longname" : "harmAmp/0",
							"parameter_mmax" : 1.0,
							"parameter_modmode" : 3,
							"parameter_shortname" : "slider[1]",
							"parameter_type" : 0
						}

					}
,
					"size" : 1.0,
					"varname" : "slider[1]"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-133",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 74.499999046325684, 1044.449575185775757, 30.0, 22.0 ],
					"text" : "2"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-131",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 39.717391014099121, 1044.449575185775757, 30.0, 22.0 ],
					"text" : "1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-127",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 39.717391014099121, 1077.058270215988159, 53.0, 22.0 ],
					"text" : "s seqOn"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-124",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 718.0, 135.0, 51.0, 22.0 ],
					"text" : "r seqOn"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-125",
					"maxclass" : "newobj",
					"numinlets" : 3,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 718.0, 162.0, 51.0, 22.0 ],
					"text" : "switch"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-122",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 639.0, 135.0, 51.0, 22.0 ],
					"text" : "r seqOn"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-123",
					"maxclass" : "newobj",
					"numinlets" : 3,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 639.0, 162.0, 51.0, 22.0 ],
					"text" : "switch"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-120",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 561.0, 135.0, 51.0, 22.0 ],
					"text" : "r seqOn"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-121",
					"maxclass" : "newobj",
					"numinlets" : 3,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 561.0, 162.0, 51.0, 22.0 ],
					"text" : "switch"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-116",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 481.0, 135.0, 51.0, 22.0 ],
					"text" : "r seqOn"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-117",
					"maxclass" : "newobj",
					"numinlets" : 3,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 481.0, 162.0, 51.0, 22.0 ],
					"text" : "switch"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-114",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 403.0, 135.0, 51.0, 22.0 ],
					"text" : "r seqOn"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-115",
					"maxclass" : "newobj",
					"numinlets" : 3,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 403.0, 162.0, 51.0, 22.0 ],
					"text" : "switch"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-112",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 323.0, 135.0, 51.0, 22.0 ],
					"text" : "r seqOn"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-113",
					"maxclass" : "newobj",
					"numinlets" : 3,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 323.0, 162.0, 51.0, 22.0 ],
					"text" : "switch"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-110",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 244.0, 135.0, 51.0, 22.0 ],
					"text" : "r seqOn"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-111",
					"maxclass" : "newobj",
					"numinlets" : 3,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 244.0, 162.0, 51.0, 22.0 ],
					"text" : "switch"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-108",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 167.0, 135.0, 51.0, 22.0 ],
					"text" : "r seqOn"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-109",
					"maxclass" : "newobj",
					"numinlets" : 3,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 167.0, 162.0, 51.0, 22.0 ],
					"text" : "switch"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-99",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 720.0, 111.0, 32.0, 22.0 ],
					"text" : "r st8"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-100",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 639.0, 111.0, 32.0, 22.0 ],
					"text" : "r st7"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-101",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 561.0, 111.0, 32.0, 22.0 ],
					"text" : "r st6"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-102",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 481.0, 111.0, 32.0, 22.0 ],
					"text" : "r st5"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-103",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 403.0, 111.0, 32.0, 22.0 ],
					"text" : "r st4"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-104",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 324.0, 111.0, 32.0, 22.0 ],
					"text" : "r st3"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-105",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 244.0, 111.0, 32.0, 22.0 ],
					"text" : "r st2"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-106",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 167.0, 111.0, 32.0, 22.0 ],
					"text" : "r st1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-107",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 87.0, 111.0, 32.0, 22.0 ],
					"text" : "r st0"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-98",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 383.195645332336426, 1514.014783620834351, 34.0, 22.0 ],
					"text" : "s st8"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-97",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 339.717385292053223, 1514.014783620834351, 34.0, 22.0 ],
					"text" : "s st7"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-96",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 294.065212249755859, 1514.014783620834351, 34.0, 22.0 ],
					"text" : "s st6"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-95",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 248.413039207458496, 1514.014783620834351, 34.0, 22.0 ],
					"text" : "s st5"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-94",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 196.239127159118652, 1514.014783620834351, 34.0, 22.0 ],
					"text" : "s st4"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-93",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 148.413041114807129, 1514.014783620834351, 34.0, 22.0 ],
					"text" : "s st3"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-92",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 111.456520080566406, 1514.014783620834351, 34.0, 22.0 ],
					"text" : "s st2"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-91",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 67.978260040283203, 1514.014783620834351, 34.0, 22.0 ],
					"text" : "s st1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-86",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 31.02173900604248, 1514.014783620834351, 34.0, 22.0 ],
					"text" : "s st0"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-85",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 87.0, 137.0, 51.0, 22.0 ],
					"text" : "r seqOn"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-84",
					"maxclass" : "newobj",
					"numinlets" : 3,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 87.0, 162.0, 51.0, 22.0 ],
					"text" : "switch"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-76",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 94.065216064453125, 1222.71044135093689, 42.0, 22.0 ],
					"text" : "$1 0 1"
				}

			}
, 			{
				"box" : 				{
					"autosize" : 1,
					"columns" : 16,
					"dialmode" : 1,
					"horizontalspacing" : 4,
					"id" : "obj-66",
					"maxclass" : "matrixctrl",
					"numinlets" : 1,
					"numoutlets" : 2,
					"one/column" : 1,
					"one/matrix" : 1,
					"one/row" : 1,
					"outlettype" : [ "list", "list" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 39.717391014099121, 1450.971306562423706, 322.0, 22.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 499.999995231628418, 203.571426630020142, 322.0, 22.0 ],
					"rows" : 1,
					"style" : "rnbohighcontrast",
					"verticalspacing" : 4
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-36",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 9,
					"outlettype" : [ "float", "float", "float", "float", "float", "float", "float", "float", "float" ],
					"patching_rect" : [ 39.717391014099121, 1483.580001592636108, 363.0, 22.0 ],
					"text" : "unpack f f f f f f f f f"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-33",
					"maxclass" : "number",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 35.369565010070801, 1222.71044135093689, 50.0, 22.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-14",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 3,
					"outlettype" : [ "signal", "signal", "int" ],
					"patching_rect" : [ 39.717391014099121, 1192.275659322738647, 67.0, 22.0 ],
					"text" : "subdiv~ 16"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-28",
					"maxclass" : "toggle",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "int" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 252.760865211486816, 1018.362619161605835, 24.0, 24.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-24",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 9,
					"outlettype" : [ "int", "int", "float", "float", "float", "", "int", "float", "" ],
					"patching_rect" : [ 179.304344177246094, 1084.415574073791504, 103.0, 22.0 ],
					"text" : "transport"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-26",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "signal" ],
					"patching_rect" : [ 39.717391014099121, 1164.014790296554565, 116.0, 22.0 ],
					"text" : "phasor~ 1n @lock 1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-30",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 207.108692169189453, 1231.40609335899353, 81.0, 22.0 ],
					"text" : "getcolumn $1"
				}

			}
, 			{
				"box" : 				{
					"autosize" : 1,
					"columns" : 16,
					"dialmode" : 2,
					"horizontalspacing" : 4,
					"id" : "obj-32",
					"maxclass" : "matrixctrl",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "list", "list" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 39.717391014099121, 1264.014788389205933, 322.0, 182.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 499.999995231628418, 17.857142686843872, 322.0, 182.0 ],
					"rows" : 9,
					"style" : "rnbohighcontrast",
					"verticalspacing" : 4
				}

			}
, 			{
				"box" : 				{
					"attr" : "tempo",
					"id" : "obj-39",
					"maxclass" : "attrui",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 183.195649147033691, 1053.145227193832397, 150.0, 22.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 560.0, 322.023231267929077, 150.0, 22.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-221",
					"linecount" : 5,
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 33.0, 108.0, 25.0, 76.0 ],
					"text" : "s ampFund"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-218",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 37.242423057556152, 412.12119197845459, 41.0, 22.0 ],
					"text" : "s fund"
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-216",
					"maxclass" : "dial",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 27.046263337135315, 304.040389180183411, 40.0, 40.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 14.285714149475098, 360.900318920612335, 36.082472205162048, 36.082472205162048 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_longname" : "fundCoarse",
							"parameter_mmax" : 1.0,
							"parameter_modmode" : 3,
							"parameter_shortname" : "dial",
							"parameter_type" : 0
						}

					}
,
					"size" : 1.0,
					"varname" : "dial[10]"
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-209",
					"maxclass" : "dial",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 78.656562447547913, 304.040389180183411, 40.0, 40.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 54.166664600372314, 360.900318920612335, 36.082472205162048, 36.082472205162048 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_longname" : "fundFine",
							"parameter_mmax" : 1.0,
							"parameter_modmode" : 3,
							"parameter_shortname" : "dial",
							"parameter_type" : 0
						}

					}
,
					"size" : 1.0,
					"varname" : "dial[9]"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-214",
					"maxclass" : "newobj",
					"numinlets" : 3,
					"numoutlets" : 2,
					"outlettype" : [ "signal", "signal" ],
					"patching_rect" : [ 30.171716332435608, 356.565639138221741, 50.0, 22.0 ],
					"text" : "fundSet"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-215",
					"linecount" : 4,
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 94.5, 354.683539390563965, 33.0, 62.0 ],
					"text" : "r ampFund"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-175",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 176.0, 308.860755443572998, 39.0, 22.0 ],
					"text" : "r fund"
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-176",
					"maxclass" : "dial",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 230.0, 276.0, 23.0, 23.0 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_longname" : "freqFine/0",
							"parameter_mmax" : 1.0,
							"parameter_modmode" : 3,
							"parameter_shortname" : "dial",
							"parameter_type" : 0
						}

					}
,
					"size" : 1.0,
					"varname" : "dial[5]"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-178",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 206.0, 275.0, 24.0, 24.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 33.665559411048889, 302.643386006355286, 16.857143610715866, 16.857143610715866 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_enum" : [ "off", "on" ],
							"parameter_longname" : "incHarm/0",
							"parameter_mmax" : 1,
							"parameter_modmode" : 0,
							"parameter_shortname" : "button[1]",
							"parameter_type" : 2
						}

					}
,
					"varname" : "button[10]"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-179",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 176.0, 275.0, 24.0, 24.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 14.285714149475098, 302.643386006355286, 16.857143610715866, 16.857143610715866 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_enum" : [ "off", "on" ],
							"parameter_longname" : "decHarm/0",
							"parameter_mmax" : 1,
							"parameter_modmode" : 0,
							"parameter_shortname" : "button",
							"parameter_type" : 2
						}

					}
,
					"varname" : "button[11]"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-181",
					"maxclass" : "newobj",
					"numinlets" : 5,
					"numoutlets" : 3,
					"outlettype" : [ "signal", "float", "int" ],
					"patching_rect" : [ 176.0, 335.0, 144.0, 22.0 ],
					"text" : "harmModTouch @harm 2",
					"varname" : "Osc0"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-183",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 732.911382794380188, 460.759487628936768, 39.0, 22.0 ],
					"text" : "r fund"
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-184",
					"maxclass" : "dial",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 791.139230132102966, 426.582272887229919, 23.0, 23.0 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_longname" : "freqFine/8",
							"parameter_mmax" : 1.0,
							"parameter_modmode" : 3,
							"parameter_shortname" : "dial",
							"parameter_type" : 0
						}

					}
,
					"size" : 1.0,
					"varname" : "dial[6]"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 12.0,
					"format" : 6,
					"id" : "obj-185",
					"maxclass" : "flonum",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 791.139230132102966, 526.582271575927734, 51.0, 22.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 329.789595007896423, 322.023231267929077, 37.113399982452393, 22.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-186",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 762.025306463241577, 426.582272887229919, 24.0, 24.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 349.944634079933167, 302.643386006355286, 16.857143610715866, 16.857143610715866 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_enum" : [ "off", "on" ],
							"parameter_longname" : "incHarm/8",
							"parameter_mmax" : 1,
							"parameter_modmode" : 0,
							"parameter_shortname" : "button[1]",
							"parameter_type" : 2
						}

					}
,
					"varname" : "button[12]"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-187",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 734.177205562591553, 426.582272887229919, 24.0, 24.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 330.564788818359375, 302.643386006355286, 16.857143610715866, 16.857143610715866 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_enum" : [ "off", "on" ],
							"parameter_longname" : "decHarm/8",
							"parameter_mmax" : 1,
							"parameter_modmode" : 0,
							"parameter_shortname" : "button",
							"parameter_type" : 2
						}

					}
,
					"varname" : "button[13]"
				}

			}
, 			{
				"box" : 				{
					"format" : 6,
					"id" : "obj-188",
					"maxclass" : "flonum",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 732.911382794380188, 526.582271575927734, 50.0, 22.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-190",
					"linecount" : 2,
					"maxclass" : "newobj",
					"numinlets" : 5,
					"numoutlets" : 3,
					"outlettype" : [ "signal", "float", "int" ],
					"patching_rect" : [ 732.911382794380188, 488.607588529586792, 105.0, 35.0 ],
					"text" : "harmModTouch @harm 13"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-191",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 786.075939059257507, 460.759487628936768, 46.0, 22.0 ],
					"text" : "r amp8"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-192",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 608.860751509666443, 460.759487628936768, 39.0, 22.0 ],
					"text" : "r fund"
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-193",
					"maxclass" : "dial",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 664.556953310966492, 426.582272887229919, 23.0, 23.0 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_longname" : "freqFine/7",
							"parameter_mmax" : 1.0,
							"parameter_modmode" : 3,
							"parameter_shortname" : "dial",
							"parameter_type" : 0
						}

					}
,
					"size" : 1.0,
					"varname" : "dial[7]"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 12.0,
					"format" : 6,
					"id" : "obj-194",
					"maxclass" : "flonum",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 662.025307774543762, 525.31644880771637, 51.0, 22.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 290.254710674285889, 322.023231267929077, 37.113399982452393, 22.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-195",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 637.974675178527832, 426.582272887229919, 24.0, 24.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 309.458657532930374, 302.643386006355286, 16.857143610715866, 16.857143610715866 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_enum" : [ "off", "on" ],
							"parameter_longname" : "incHarm/7",
							"parameter_mmax" : 1,
							"parameter_modmode" : 0,
							"parameter_shortname" : "button[1]",
							"parameter_type" : 2
						}

					}
,
					"varname" : "button[14]"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-196",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 613.924042582511902, 426.582272887229919, 24.0, 24.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 290.254710674285889, 302.643386006355286, 16.857143610715866, 16.857143610715866 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_enum" : [ "off", "on" ],
							"parameter_longname" : "decHarm/7",
							"parameter_mmax" : 1,
							"parameter_modmode" : 0,
							"parameter_shortname" : "button",
							"parameter_type" : 2
						}

					}
,
					"varname" : "button[15]"
				}

			}
, 			{
				"box" : 				{
					"format" : 6,
					"id" : "obj-197",
					"maxclass" : "flonum",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 608.860751509666443, 525.31644880771637, 50.0, 22.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-198",
					"linecount" : 2,
					"maxclass" : "newobj",
					"numinlets" : 5,
					"numoutlets" : 3,
					"outlettype" : [ "signal", "float", "int" ],
					"patching_rect" : [ 608.860751509666443, 488.607588529586792, 105.0, 35.0 ],
					"text" : "harmModTouch @harm 11"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-199",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 662.025307774543762, 460.759487628936768, 46.0, 22.0 ],
					"text" : "r amp7"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-200",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 481.012651920318604, 460.759487628936768, 39.0, 22.0 ],
					"text" : "r fund"
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-201",
					"maxclass" : "dial",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 537.974676489830017, 426.582272887229919, 23.0, 23.0 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_longname" : "freqFine/6",
							"parameter_mmax" : 1.0,
							"parameter_modmode" : 3,
							"parameter_shortname" : "dial",
							"parameter_type" : 0
						}

					}
,
					"size" : 1.0,
					"varname" : "dial[8]"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 12.0,
					"format" : 6,
					"id" : "obj-202",
					"maxclass" : "flonum",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 537.974676489830017, 526.582271575927734, 51.0, 22.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 251.495020151138306, 322.023231267929077, 37.113399982452393, 22.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-203",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 511.392398357391357, 426.582272887229919, 24.0, 24.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 270.874865412712097, 302.643386006355286, 16.857143610715866, 16.857143610715866 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_enum" : [ "off", "on" ],
							"parameter_longname" : "incHarm/6",
							"parameter_mmax" : 1,
							"parameter_modmode" : 0,
							"parameter_shortname" : "button[1]",
							"parameter_type" : 2
						}

					}
,
					"varname" : "button[16]"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-204",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 481.012651920318604, 426.582272887229919, 24.0, 24.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 251.495020151138306, 302.643386006355286, 16.857143610715866, 16.857143610715866 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_enum" : [ "off", "on" ],
							"parameter_longname" : "decHarm/6",
							"parameter_mmax" : 1,
							"parameter_modmode" : 0,
							"parameter_shortname" : "button",
							"parameter_type" : 2
						}

					}
,
					"varname" : "button[17]"
				}

			}
, 			{
				"box" : 				{
					"format" : 6,
					"id" : "obj-205",
					"maxclass" : "flonum",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 481.012651920318604, 526.582271575927734, 50.0, 22.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-206",
					"linecount" : 2,
					"maxclass" : "newobj",
					"numinlets" : 5,
					"numoutlets" : 3,
					"outlettype" : [ "signal", "float", "int" ],
					"patching_rect" : [ 481.012651920318604, 488.607588529586792, 105.0, 35.0 ],
					"text" : "harmModTouch @harm 10"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-207",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 534.177208185195923, 460.759487628936768, 46.0, 22.0 ],
					"text" : "r amp6"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-159",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 362.025311708450317, 464.556955933570862, 39.0, 22.0 ],
					"text" : "r fund"
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-160",
					"maxclass" : "dial",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 417.721513509750366, 429.113918423652649, 23.0, 23.0 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_longname" : "freqFine/5",
							"parameter_mmax" : 1.0,
							"parameter_modmode" : 3,
							"parameter_shortname" : "dial",
							"parameter_type" : 0
						}

					}
,
					"size" : 1.0,
					"varname" : "dial[4]"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-162",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 391.139235377311707, 429.113918423652649, 24.0, 24.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 230.511286288499832, 302.643386006355286, 16.857143610715866, 16.857143610715866 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_enum" : [ "off", "on" ],
							"parameter_longname" : "incHarm/5",
							"parameter_mmax" : 1,
							"parameter_modmode" : 0,
							"parameter_shortname" : "button[1]",
							"parameter_type" : 2
						}

					}
,
					"varname" : "button[8]"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-163",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 362.025311708450317, 429.113918423652649, 24.0, 24.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 211.184942007064819, 302.643386006355286, 16.857143610715866, 16.857143610715866 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_enum" : [ "off", "on" ],
							"parameter_longname" : "decHarm/5",
							"parameter_mmax" : 1,
							"parameter_modmode" : 0,
							"parameter_shortname" : "button",
							"parameter_type" : 2
						}

					}
,
					"varname" : "button[9]"
				}

			}
, 			{
				"box" : 				{
					"format" : 6,
					"id" : "obj-164",
					"maxclass" : "flonum",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 362.025311708450317, 529.113917112350464, 50.0, 22.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-165",
					"linecount" : 2,
					"maxclass" : "newobj",
					"numinlets" : 5,
					"numoutlets" : 3,
					"outlettype" : [ "signal", "float", "int" ],
					"patching_rect" : [ 362.025311708450317, 489.873411297798157, 99.0, 35.0 ],
					"text" : "harmModTouch @harm 9"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-166",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 413.924045205116272, 464.556955933570862, 46.0, 22.0 ],
					"text" : "r amp5"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-150",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 732.911382794380188, 310.126578211784363, 39.0, 22.0 ],
					"text" : "r fund"
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-151",
					"maxclass" : "dial",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 791.139230132102966, 275.949363470077515, 23.0, 23.0 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_longname" : "freqFine/4",
							"parameter_mmax" : 1.0,
							"parameter_modmode" : 3,
							"parameter_shortname" : "dial",
							"parameter_type" : 0
						}

					}
,
					"size" : 1.0,
					"varname" : "dial[3]"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-153",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 762.025306463241577, 274.68354070186615, 24.0, 24.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 191.037600666284561, 302.643386006355286, 16.857143610715866, 16.857143610715866 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_enum" : [ "off", "on" ],
							"parameter_longname" : "incHarm/4",
							"parameter_mmax" : 1,
							"parameter_modmode" : 0,
							"parameter_shortname" : "button[1]",
							"parameter_type" : 2
						}

					}
,
					"varname" : "button[6]"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-154",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 732.911382794380188, 274.68354070186615, 24.0, 24.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 172.425251483917236, 302.643386006355286, 16.857143610715866, 16.857143610715866 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_enum" : [ "off", "on" ],
							"parameter_longname" : "decHarm/4",
							"parameter_mmax" : 1,
							"parameter_modmode" : 0,
							"parameter_shortname" : "button",
							"parameter_type" : 2
						}

					}
,
					"varname" : "button[7]"
				}

			}
, 			{
				"box" : 				{
					"format" : 6,
					"id" : "obj-155",
					"maxclass" : "flonum",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 732.911382794380188, 375.94936215877533, 50.0, 22.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-156",
					"linecount" : 2,
					"maxclass" : "newobj",
					"numinlets" : 5,
					"numoutlets" : 3,
					"outlettype" : [ "signal", "float", "int" ],
					"patching_rect" : [ 732.911382794380188, 339.240501880645752, 99.0, 35.0 ],
					"text" : "harmModTouch @harm 8"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-158",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 786.075939059257507, 310.126578211784363, 46.0, 22.0 ],
					"text" : "r amp4"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-142",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 608.860751509666443, 308.860755443572998, 39.0, 22.0 ],
					"text" : "r fund"
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-143",
					"maxclass" : "dial",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 664.556953310966492, 275.949363470077515, 23.0, 23.0 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_longname" : "freqFine/3",
							"parameter_mmax" : 1.0,
							"parameter_modmode" : 3,
							"parameter_shortname" : "dial",
							"parameter_type" : 0
						}

					}
,
					"size" : 1.0,
					"varname" : "dial[2]"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-145",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 636.708852410316467, 274.68354070186615, 24.0, 24.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 151.56391504406929, 302.643386006355286, 16.857143610715866, 16.857143610715866 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_enum" : [ "off", "on" ],
							"parameter_longname" : "incHarm/3",
							"parameter_mmax" : 1,
							"parameter_modmode" : 0,
							"parameter_shortname" : "button[1]",
							"parameter_type" : 2
						}

					}
,
					"varname" : "button[4]"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-146",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 608.860751509666443, 274.68354070186615, 24.0, 24.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 132.11517333984375, 302.643386006355286, 16.857143610715866, 16.857143610715866 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_enum" : [ "off", "on" ],
							"parameter_longname" : "decHarm/3",
							"parameter_mmax" : 1,
							"parameter_modmode" : 0,
							"parameter_shortname" : "button",
							"parameter_type" : 2
						}

					}
,
					"varname" : "button[5]"
				}

			}
, 			{
				"box" : 				{
					"format" : 6,
					"id" : "obj-147",
					"maxclass" : "flonum",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 608.860751509666443, 379.44936215877533, 50.0, 22.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-148",
					"linecount" : 2,
					"maxclass" : "newobj",
					"numinlets" : 5,
					"numoutlets" : 3,
					"outlettype" : [ "signal", "float", "int" ],
					"patching_rect" : [ 608.860751509666443, 336.708856344223022, 99.0, 35.0 ],
					"text" : "harmModTouch @harm 7"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-149",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 656.962016701698303, 308.860755443572998, 46.0, 22.0 ],
					"text" : "r amp3"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-134",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 481.012651920318604, 308.860755443572998, 39.0, 22.0 ],
					"text" : "r fund"
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-135",
					"maxclass" : "dial",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 536.708853721618652, 275.949363470077515, 23.0, 23.0 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_longname" : "freqFine/2",
							"parameter_mmax" : 1.0,
							"parameter_modmode" : 3,
							"parameter_shortname" : "dial",
							"parameter_type" : 0
						}

					}
,
					"size" : 1.0,
					"varname" : "dial[1]"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-137",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 508.860752820968628, 274.68354070186615, 24.0, 24.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 112.090229421854019, 302.643386006355286, 16.857143610715866, 16.857143610715866 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_enum" : [ "off", "on" ],
							"parameter_longname" : "incHarm/2",
							"parameter_mmax" : 1,
							"parameter_modmode" : 0,
							"parameter_shortname" : "button[1]",
							"parameter_type" : 2
						}

					}
,
					"varname" : "button[2]"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-138",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 481.012651920318604, 274.68354070186615, 24.0, 24.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 93.333336114883423, 302.643386006355286, 16.857143610715866, 16.857143610715866 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_enum" : [ "off", "on" ],
							"parameter_longname" : "decHarm/2",
							"parameter_mmax" : 1,
							"parameter_modmode" : 0,
							"parameter_shortname" : "button",
							"parameter_type" : 2
						}

					}
,
					"varname" : "button[3]"
				}

			}
, 			{
				"box" : 				{
					"format" : 6,
					"id" : "obj-139",
					"maxclass" : "flonum",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 481.012651920318604, 375.94936215877533, 50.0, 22.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-140",
					"linecount" : 2,
					"maxclass" : "newobj",
					"numinlets" : 5,
					"numoutlets" : 3,
					"outlettype" : [ "signal", "float", "int" ],
					"patching_rect" : [ 481.012651920318604, 336.708856344223022, 99.0, 35.0 ],
					"text" : "harmModTouch @harm 5"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-141",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 532.911385416984558, 308.860755443572998, 46.0, 22.0 ],
					"text" : "r amp2"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-11",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 362.025311708450317, 308.860755443572998, 39.0, 22.0 ],
					"text" : "r fund"
				}

			}
, 			{
				"box" : 				{
					"floatoutput" : 1,
					"id" : "obj-74",
					"maxclass" : "dial",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 417.721513509750366, 275.949363470077515, 23.0, 23.0 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_longname" : "freqFine/1",
							"parameter_mmax" : 1.0,
							"parameter_modmode" : 3,
							"parameter_shortname" : "dial",
							"parameter_type" : 0
						}

					}
,
					"size" : 1.0,
					"varname" : "dial"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-71",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 391.139235377311707, 274.68354070186615, 24.0, 24.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 72.425249934196472, 302.643386006355286, 16.857143610715866, 16.857143610715866 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_enum" : [ "off", "on" ],
							"parameter_longname" : "incHarm/1",
							"parameter_mmax" : 1,
							"parameter_modmode" : 0,
							"parameter_shortname" : "button[1]",
							"parameter_type" : 2
						}

					}
,
					"varname" : "button[1]"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-69",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 362.025311708450317, 274.68354070186615, 24.0, 24.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 53.045404672622681, 302.643386006355286, 16.857143610715866, 16.857143610715866 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_enum" : [ "off", "on" ],
							"parameter_longname" : "decHarm/1",
							"parameter_mmax" : 1,
							"parameter_modmode" : 0,
							"parameter_shortname" : "button",
							"parameter_type" : 2
						}

					}
,
					"varname" : "button"
				}

			}
, 			{
				"box" : 				{
					"format" : 6,
					"id" : "obj-46",
					"maxclass" : "flonum",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 362.025311708450317, 374.683539390563965, 50.0, 22.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-43",
					"linecount" : 2,
					"maxclass" : "newobj",
					"numinlets" : 5,
					"numoutlets" : 3,
					"outlettype" : [ "signal", "float", "int" ],
					"patching_rect" : [ 362.025311708450317, 336.708856344223022, 99.0, 35.0 ],
					"text" : "harmModTouch @harm 3"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-65",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 413.924045205116272, 308.860755443572998, 46.0, 22.0 ],
					"text" : "r amp1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-64",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 718.0, 186.0, 48.0, 22.0 ],
					"text" : "s amp8"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-63",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 639.0, 186.0, 48.0, 22.0 ],
					"text" : "s amp7"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-62",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 561.0, 186.0, 48.0, 22.0 ],
					"text" : "s amp6"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-61",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 481.0, 186.0, 48.0, 22.0 ],
					"text" : "s amp5"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-60",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 403.0, 186.0, 48.0, 22.0 ],
					"text" : "s amp4"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-59",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 323.0, 186.0, 48.0, 22.0 ],
					"text" : "s amp3"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-58",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 244.0, 186.0, 48.0, 22.0 ],
					"text" : "s amp2"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-57",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 167.0, 186.0, 48.0, 22.0 ],
					"text" : "s amp1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-55",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 18.309859395027161, 204.92958015203476, 48.0, 22.0 ],
					"text" : "s amp0"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-4",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"patching_rect" : [ 61.484846115112305, 272.727259397506714, 58.0, 22.0 ],
					"text" : "loadbang"
				}

			}
, 			{
				"box" : 				{
					"dbperled" : 1,
					"id" : "obj-29",
					"maxclass" : "meter~",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"patching_rect" : [ 33.846157073974609, 690.230834007263184, 77.0, 22.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 444.186053395271301, 14.271288514137268, 44.155843734741211, 329.870126724243164 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-22",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "signal" ],
					"patching_rect" : [ 33.846157073974609, 627.153904914855957, 34.0, 22.0 ],
					"text" : "*~ 1."
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-20",
					"maxclass" : "newobj",
					"numinlets" : 3,
					"numoutlets" : 1,
					"outlettype" : [ "signal" ],
					"patching_rect" : [ 33.846157073974609, 656.384676933288574, 64.0, 22.0 ],
					"text" : "clip~ -1. 1."
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-157",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "float", "float" ],
					"patching_rect" : [ 176.814815998077393, 691.35807991027832, 39.0, 22.0 ],
					"text" : "t f f"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-37",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"patching_rect" : [ 223.728399991989136, 630.864247918128967, 58.0, 22.0 ],
					"text" : "loadbang"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-34",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 4,
					"outlettype" : [ "int", "float", "int", "int" ],
					"patching_rect" : [ 223.728399991989136, 661.728447914123535, 61.0, 22.0 ],
					"text" : "dspstate~"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-17",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"patching_rect" : [ 223.728399991989136, 724.691415905952454, 33.0, 22.0 ],
					"text" : "* 10."
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-18",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"patching_rect" : [ 223.728399991989136, 692.592647910118103, 55.0, 22.0 ],
					"text" : "!/ 44100."
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-19",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 286.691367983818054, 734.567959904670715, 70.0, 22.0 ],
					"text" : "samples $1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-16",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"patching_rect" : [ 505.209903955459595, 709.876599907875061, 30.0, 22.0 ],
					"text" : "/ 4."
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-15",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "signal" ],
					"patching_rect" : [ 505.209903955459595, 734.567959904670715, 43.0, 22.0 ],
					"text" : "cycle~"
				}

			}
, 			{
				"box" : 				{
					"grid" : 2,
					"id" : "obj-6",
					"interval" : 51.0,
					"maxclass" : "live.scope~",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"patching_rect" : [ 281.209903955459595, 806.666640222072601, 267.0, 139.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 14.285714149475098, 14.271288514137268, 352.559687756001949, 179.22077751159668 ],
					"range" : [ -1.0, 1.01 ],
					"samples" : 7939.512572991444358
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-1",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 61.346160888671875, 729.251693725585938, 45.0, 22.0 ],
					"text" : "dac~ 3"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-2",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "signal" ],
					"patching_rect" : [ 33.846157073974609, 597.92313289642334, 30.0, 22.0 ],
					"text" : "+~"
				}

			}
, 			{
				"box" : 				{
					"attr" : "range",
					"id" : "obj-9",
					"maxclass" : "attrui",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 286.691367983818054, 703.703759908676147, 177.0, 22.0 ]
				}

			}
, 			{
				"box" : 				{
					"attr" : "interval",
					"id" : "obj-10",
					"maxclass" : "attrui",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 286.691367983818054, 674.074127912521362, 150.0, 22.0 ]
				}

			}
, 			{
				"box" : 				{
					"attr" : "mode",
					"id" : "obj-12",
					"maxclass" : "attrui",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 286.691367983818054, 644.444495916366577, 150.0, 22.0 ]
				}

			}
, 			{
				"box" : 				{
					"angle" : 270.0,
					"bgcolor" : [ 0.494117647058824, 0.780392156862745, 0.796078431372549, 1.0 ],
					"id" : "obj-173",
					"maxclass" : "panel",
					"mode" : 0,
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 164.705889225006104, 247.058833837509155, 837.647093772888184, 351.764720559120178 ],
					"proportion" : 0.39
				}

			}
, 			{
				"box" : 				{
					"angle" : 270.0,
					"bgcolor" : [ 0.494117647058824, 0.780392156862745, 0.796078431372549, 1.0 ],
					"id" : "obj-189",
					"maxclass" : "panel",
					"mode" : 0,
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 13.698629140853882, 5.479451656341553, 759.885779619216919, 212.328751683235168 ],
					"proportion" : 0.39
				}

			}
, 			{
				"box" : 				{
					"angle" : 270.0,
					"bgcolor" : [ 0.494117647058824, 0.780392156862745, 0.796078431372549, 1.0 ],
					"id" : "obj-222",
					"maxclass" : "panel",
					"mode" : 0,
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 24.5, 955.31914210319519, 327.0, 181.0 ],
					"proportion" : 0.39
				}

			}
, 			{
				"box" : 				{
					"angle" : 270.0,
					"bgcolor" : [ 0.494117647058824, 0.780392156862745, 0.796078431372549, 1.0 ],
					"id" : "obj-223",
					"maxclass" : "panel",
					"mode" : 0,
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 24.5, 1146.623486280441284, 401.234599947929382, 404.93830394744873 ],
					"proportion" : 0.39
				}

			}
, 			{
				"box" : 				{
					"angle" : 270.0,
					"bgcolor" : [ 0.494117647058824, 0.780392156862745, 0.796078431372549, 1.0 ],
					"id" : "obj-224",
					"maxclass" : "panel",
					"mode" : 0,
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 162.0, 608.642023921012878, 408.0, 319.0 ],
					"proportion" : 0.39
				}

			}
, 			{
				"box" : 				{
					"angle" : 270.0,
					"bgcolor" : [ 0.494117647058824, 0.780392156862745, 0.796078431372549, 1.0 ],
					"id" : "obj-225",
					"maxclass" : "panel",
					"mode" : 0,
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 13.0, 245.0, 129.0, 238.0 ],
					"proportion" : 0.39
				}

			}
, 			{
				"box" : 				{
					"angle" : 270.0,
					"bgcolor" : [ 0.494117647058824, 0.780392156862745, 0.796078431372549, 1.0 ],
					"id" : "obj-226",
					"maxclass" : "panel",
					"mode" : 0,
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 18.461540222167969, 487.153891563415527, 109.230779647827148, 272.307718276977539 ],
					"proportion" : 0.39
				}

			}
 ],
		"lines" : [ 			{
				"patchline" : 				{
					"destination" : [ "obj-6", 0 ],
					"midpoints" : [ 296.191367983818054, 699.0, 282.0, 699.0, 282.0, 759.0, 290.709903955459595, 759.0 ],
					"source" : [ "obj-10", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-123", 2 ],
					"midpoints" : [ 648.5, 135.0, 636.0, 135.0, 636.0, 159.0, 680.5, 159.0 ],
					"source" : [ "obj-100", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-121", 2 ],
					"midpoints" : [ 570.5, 135.0, 558.0, 135.0, 558.0, 159.0, 602.5, 159.0 ],
					"source" : [ "obj-101", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-117", 2 ],
					"midpoints" : [ 490.5, 135.0, 477.0, 135.0, 477.0, 159.0, 522.5, 159.0 ],
					"source" : [ "obj-102", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-115", 2 ],
					"midpoints" : [ 412.5, 135.0, 399.0, 135.0, 399.0, 159.0, 444.5, 159.0 ],
					"source" : [ "obj-103", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-113", 2 ],
					"midpoints" : [ 333.5, 135.0, 318.0, 135.0, 318.0, 159.0, 364.5, 159.0 ],
					"source" : [ "obj-104", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-111", 2 ],
					"midpoints" : [ 253.5, 135.0, 240.0, 135.0, 240.0, 159.0, 285.5, 159.0 ],
					"source" : [ "obj-105", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-109", 2 ],
					"midpoints" : [ 176.5, 135.0, 162.0, 135.0, 162.0, 159.0, 208.5, 159.0 ],
					"source" : [ "obj-106", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-84", 2 ],
					"midpoints" : [ 96.5, 135.0, 84.0, 135.0, 84.0, 159.0, 128.5, 159.0 ],
					"source" : [ "obj-107", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-109", 0 ],
					"midpoints" : [ 176.5, 159.0, 176.5, 159.0 ],
					"order" : 0,
					"source" : [ "obj-108", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-128", 0 ],
					"midpoints" : [ 176.5, 159.0, 153.0, 159.0, 153.0, 21.0, 171.0, 21.0 ],
					"order" : 1,
					"source" : [ "obj-108", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-38", 0 ],
					"midpoints" : [ 176.5, 186.0, 153.0, 186.0, 153.0, 36.0, 190.5, 36.0 ],
					"order" : 0,
					"source" : [ "obj-109", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-57", 0 ],
					"midpoints" : [ 176.5, 186.0, 176.5, 186.0 ],
					"order" : 1,
					"source" : [ "obj-109", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-43", 0 ],
					"midpoints" : [ 371.525311708450317, 333.0, 371.525311708450317, 333.0 ],
					"source" : [ "obj-11", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-111", 0 ],
					"midpoints" : [ 253.5, 159.0, 253.5, 159.0 ],
					"order" : 0,
					"source" : [ "obj-110", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-129", 0 ],
					"midpoints" : [ 253.5, 159.0, 231.0, 159.0, 231.0, 21.0, 248.0, 21.0 ],
					"order" : 1,
					"source" : [ "obj-110", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-45", 0 ],
					"midpoints" : [ 253.5, 186.0, 231.0, 186.0, 231.0, 36.0, 268.5, 36.0 ],
					"order" : 0,
					"source" : [ "obj-111", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-58", 0 ],
					"midpoints" : [ 253.5, 186.0, 253.5, 186.0 ],
					"order" : 1,
					"source" : [ "obj-111", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-113", 0 ],
					"midpoints" : [ 332.5, 159.0, 332.5, 159.0 ],
					"order" : 1,
					"source" : [ "obj-112", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-167", 0 ],
					"midpoints" : [ 332.5, 159.0, 309.0, 159.0, 309.0, 21.0, 328.0, 21.0 ],
					"order" : 0,
					"source" : [ "obj-112", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-53", 0 ],
					"midpoints" : [ 332.5, 186.0, 309.0, 186.0, 309.0, 36.0, 348.5, 36.0 ],
					"order" : 0,
					"source" : [ "obj-113", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-59", 0 ],
					"midpoints" : [ 332.5, 186.0, 332.5, 186.0 ],
					"order" : 1,
					"source" : [ "obj-113", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-115", 0 ],
					"midpoints" : [ 412.5, 159.0, 412.5, 159.0 ],
					"order" : 0,
					"source" : [ "obj-114", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-132", 0 ],
					"midpoints" : [ 412.5, 159.0, 390.0, 159.0, 390.0, 21.0, 407.0, 21.0 ],
					"order" : 1,
					"source" : [ "obj-114", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-54", 0 ],
					"midpoints" : [ 412.5, 186.0, 390.0, 186.0, 390.0, 36.0, 429.5, 36.0 ],
					"order" : 0,
					"source" : [ "obj-115", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-60", 0 ],
					"midpoints" : [ 412.5, 186.0, 412.5, 186.0 ],
					"order" : 1,
					"source" : [ "obj-115", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-117", 0 ],
					"midpoints" : [ 490.5, 159.0, 490.5, 159.0 ],
					"order" : 0,
					"source" : [ "obj-116", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-130", 0 ],
					"midpoints" : [ 490.5, 159.0, 468.0, 159.0, 468.0, 21.0, 485.0, 21.0 ],
					"order" : 1,
					"source" : [ "obj-116", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-61", 0 ],
					"midpoints" : [ 490.5, 186.0, 490.5, 186.0 ],
					"order" : 1,
					"source" : [ "obj-117", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-68", 0 ],
					"midpoints" : [ 490.5, 186.0, 468.0, 186.0, 468.0, 36.0, 506.5, 36.0 ],
					"order" : 0,
					"source" : [ "obj-117", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-119", 0 ],
					"midpoints" : [ 569.817469000816345, 1234.884467840194702, 486.976198077201843, 1234.884467840194702, 486.976198077201843, 1435.884467840194702, 514.26191258430481, 1435.884467840194702 ],
					"source" : [ "obj-118", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-6", 0 ],
					"midpoints" : [ 296.191367983818054, 669.0, 474.0, 669.0, 474.0, 750.0, 357.0, 750.0, 357.0, 762.0, 290.709903955459595, 762.0 ],
					"source" : [ "obj-12", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-121", 0 ],
					"midpoints" : [ 570.5, 159.0, 570.5, 159.0 ],
					"order" : 0,
					"source" : [ "obj-120", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-170", 0 ],
					"midpoints" : [ 570.5, 159.0, 546.0, 159.0, 546.0, 21.0, 563.0, 21.0 ],
					"order" : 1,
					"source" : [ "obj-120", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-62", 0 ],
					"midpoints" : [ 570.5, 186.0, 570.5, 186.0 ],
					"order" : 1,
					"source" : [ "obj-121", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-79", 0 ],
					"midpoints" : [ 570.5, 186.0, 546.0, 186.0, 546.0, 36.0, 586.5, 36.0 ],
					"order" : 0,
					"source" : [ "obj-121", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-123", 0 ],
					"midpoints" : [ 648.5, 159.0, 648.5, 159.0 ],
					"order" : 0,
					"source" : [ "obj-122", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-169", 0 ],
					"midpoints" : [ 648.5, 159.0, 624.0, 159.0, 624.0, 21.0, 643.0, 21.0 ],
					"order" : 1,
					"source" : [ "obj-122", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-63", 0 ],
					"midpoints" : [ 648.5, 186.0, 648.5, 186.0 ],
					"order" : 1,
					"source" : [ "obj-123", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-80", 0 ],
					"midpoints" : [ 648.5, 186.0, 624.0, 186.0, 624.0, 36.0, 663.5, 36.0 ],
					"order" : 0,
					"source" : [ "obj-123", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-125", 0 ],
					"midpoints" : [ 727.5, 159.0, 727.5, 159.0 ],
					"order" : 1,
					"source" : [ "obj-124", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-168", 0 ],
					"midpoints" : [ 727.5, 159.0, 705.0, 159.0, 705.0, 21.0, 724.0, 21.0 ],
					"order" : 0,
					"source" : [ "obj-124", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-64", 0 ],
					"midpoints" : [ 727.5, 186.0, 727.5, 186.0 ],
					"order" : 1,
					"source" : [ "obj-125", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-81", 0 ],
					"midpoints" : [ 727.5, 186.0, 705.0, 186.0, 705.0, 36.0, 742.5, 36.0 ],
					"order" : 0,
					"source" : [ "obj-125", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-31", 0 ],
					"midpoints" : [ 858.26191258430481, 1495.884467840194702, 858.706362366676331, 1495.884467840194702 ],
					"source" : [ "obj-126", 8 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-42", 0 ],
					"midpoints" : [ 815.26191258430481, 1495.884467840194702, 814.261917233467102, 1495.884467840194702 ],
					"source" : [ "obj-126", 7 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-44", 0 ],
					"midpoints" : [ 772.26191258430481, 1495.884467840194702, 769.817472100257874, 1495.884467840194702 ],
					"source" : [ "obj-126", 6 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-47", 0 ],
					"midpoints" : [ 729.26191258430481, 1495.884467840194702, 723.785725355148315, 1495.884467840194702 ],
					"source" : [ "obj-126", 5 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-48", 0 ],
					"midpoints" : [ 686.26191258430481, 1495.884467840194702, 671.404772162437439, 1495.884467840194702 ],
					"source" : [ "obj-126", 4 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-49", 0 ],
					"midpoints" : [ 643.26191258430481, 1495.884467840194702, 623.785723805427551, 1495.884467840194702 ],
					"source" : [ "obj-126", 3 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-72", 0 ],
					"midpoints" : [ 600.26191258430481, 1495.884467840194702, 587.981005072593689, 1495.884467840194702 ],
					"source" : [ "obj-126", 2 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-83", 0 ],
					"midpoints" : [ 557.26191258430481, 1495.884467840194702, 542.833341598510742, 1495.884467840194702 ],
					"source" : [ "obj-126", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-90", 0 ],
					"midpoints" : [ 514.26191258430481, 1495.884467840194702, 506.325404524803162, 1495.884467840194702 ],
					"source" : [ "obj-126", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-8", 0 ],
					"midpoints" : [ 171.0, 36.0, 172.0, 36.0 ],
					"source" : [ "obj-128", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-40", 0 ],
					"midpoints" : [ 248.0, 36.0, 249.0, 36.0 ],
					"source" : [ "obj-129", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-51", 0 ],
					"midpoints" : [ 485.0, 36.0, 486.0, 36.0 ],
					"source" : [ "obj-130", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-127", 0 ],
					"midpoints" : [ 49.217391014099121, 1068.0, 49.217391014099121, 1068.0 ],
					"source" : [ "obj-131", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-50", 0 ],
					"midpoints" : [ 407.0, 36.0, 408.0, 36.0 ],
					"source" : [ "obj-132", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-127", 0 ],
					"midpoints" : [ 83.999999046325684, 1068.0, 51.0, 1068.0, 51.0, 1074.0, 49.217391014099121, 1074.0 ],
					"source" : [ "obj-133", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-140", 0 ],
					"midpoints" : [ 490.512651920318604, 333.0, 490.512651920318604, 333.0 ],
					"source" : [ "obj-134", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-140", 3 ],
					"midpoints" : [ 546.208853721618652, 300.0, 528.0, 300.0, 528.0, 333.0, 550.512651920318604, 333.0 ],
					"source" : [ "obj-135", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-405", 0 ],
					"source" : [ "obj-136", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-140", 2 ],
					"midpoints" : [ 518.360752820968628, 300.0, 530.512651920318604, 300.0 ],
					"source" : [ "obj-137", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-140", 1 ],
					"midpoints" : [ 490.512651920318604, 300.0, 477.0, 300.0, 477.0, 333.0, 510.512651920318604, 333.0 ],
					"source" : [ "obj-138", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-33", 0 ],
					"midpoints" : [ 97.217391014099121, 1215.0, 44.869565010070801, 1215.0 ],
					"source" : [ "obj-14", 2 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-139", 0 ],
					"midpoints" : [ 530.512651920318604, 372.0, 490.512651920318604, 372.0 ],
					"source" : [ "obj-140", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-2", 0 ],
					"midpoints" : [ 490.512651920318604, 372.0, 477.0, 372.0, 477.0, 411.0, 153.0, 411.0, 153.0, 495.0, 43.346157073974609, 495.0 ],
					"source" : [ "obj-140", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-233", 0 ],
					"midpoints" : [ 570.512651920318604, 372.0, 543.411385416984558, 372.0 ],
					"source" : [ "obj-140", 2 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-140", 4 ],
					"midpoints" : [ 542.411385416984558, 333.0, 570.512651920318604, 333.0 ],
					"source" : [ "obj-141", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-148", 0 ],
					"midpoints" : [ 618.360751509666443, 333.0, 618.360751509666443, 333.0 ],
					"source" : [ "obj-142", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-148", 3 ],
					"midpoints" : [ 674.056953310966492, 300.0, 651.0, 300.0, 651.0, 333.0, 678.360751509666443, 333.0 ],
					"source" : [ "obj-143", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-148", 2 ],
					"midpoints" : [ 646.208852410316467, 303.0, 658.360751509666443, 303.0 ],
					"source" : [ "obj-145", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-148", 1 ],
					"midpoints" : [ 618.360751509666443, 300.0, 648.0, 300.0, 648.0, 333.0, 638.360751509666443, 333.0 ],
					"source" : [ "obj-146", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-147", 0 ],
					"midpoints" : [ 658.360751509666443, 372.0, 618.360751509666443, 372.0 ],
					"source" : [ "obj-148", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-2", 0 ],
					"midpoints" : [ 618.360751509666443, 372.0, 603.0, 372.0, 603.0, 411.0, 153.0, 411.0, 153.0, 495.0, 43.346157073974609, 495.0 ],
					"source" : [ "obj-148", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-234", 0 ],
					"midpoints" : [ 698.360751509666443, 372.0, 669.025307774543762, 372.0 ],
					"source" : [ "obj-148", 2 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-148", 4 ],
					"midpoints" : [ 666.462016701698303, 333.0, 698.360751509666443, 333.0 ],
					"source" : [ "obj-149", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-6", 1 ],
					"midpoints" : [ 514.709903955459595, 759.0, 538.709903955459595, 759.0 ],
					"source" : [ "obj-15", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-156", 0 ],
					"midpoints" : [ 742.411382794380188, 333.0, 742.411382794380188, 333.0 ],
					"source" : [ "obj-150", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-156", 3 ],
					"midpoints" : [ 800.639230132102966, 300.0, 783.0, 300.0, 783.0, 333.0, 802.411382794380188, 333.0 ],
					"source" : [ "obj-151", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-156", 2 ],
					"midpoints" : [ 771.525306463241577, 336.0, 782.411382794380188, 336.0 ],
					"source" : [ "obj-153", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-156", 1 ],
					"midpoints" : [ 742.411382794380188, 306.0, 729.0, 306.0, 729.0, 336.0, 762.411382794380188, 336.0 ],
					"source" : [ "obj-154", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-155", 0 ],
					"midpoints" : [ 782.411382794380188, 375.0, 783.0, 375.0, 783.0, 408.0, 729.0, 408.0, 729.0, 372.0, 742.411382794380188, 372.0 ],
					"source" : [ "obj-156", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-2", 0 ],
					"midpoints" : [ 742.411382794380188, 375.0, 729.0, 375.0, 729.0, 411.0, 153.0, 411.0, 153.0, 495.0, 43.346157073974609, 495.0 ],
					"source" : [ "obj-156", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-235", 0 ],
					"midpoints" : [ 822.411382794380188, 375.0, 805.33331435918808, 375.0 ],
					"source" : [ "obj-156", 2 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-16", 0 ],
					"midpoints" : [ 206.314815998077393, 756.0, 366.0, 756.0, 366.0, 735.0, 492.0, 735.0, 492.0, 705.0, 514.709903955459595, 705.0 ],
					"source" : [ "obj-157", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-18", 0 ],
					"midpoints" : [ 186.314815998077393, 714.0, 219.0, 714.0, 219.0, 687.0, 233.228399991989136, 687.0 ],
					"source" : [ "obj-157", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-156", 4 ],
					"midpoints" : [ 795.575939059257507, 333.0, 822.411382794380188, 333.0 ],
					"source" : [ "obj-158", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-165", 0 ],
					"midpoints" : [ 371.525311708450317, 489.0, 371.525311708450317, 489.0 ],
					"source" : [ "obj-159", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-15", 0 ],
					"midpoints" : [ 514.709903955459595, 732.0, 514.709903955459595, 732.0 ],
					"source" : [ "obj-16", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-165", 3 ],
					"midpoints" : [ 427.221513509750366, 453.0, 408.0, 453.0, 408.0, 486.0, 431.525311708450317, 486.0 ],
					"source" : [ "obj-160", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-165", 2 ],
					"midpoints" : [ 400.639235377311707, 459.0, 411.525311708450317, 459.0 ],
					"source" : [ "obj-162", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-165", 1 ],
					"midpoints" : [ 371.525311708450317, 456.0, 357.0, 456.0, 357.0, 486.0, 391.525311708450317, 486.0 ],
					"source" : [ "obj-163", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-164", 0 ],
					"midpoints" : [ 411.525311708450317, 525.0, 371.525311708450317, 525.0 ],
					"source" : [ "obj-165", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-2", 0 ],
					"midpoints" : [ 371.525311708450317, 525.0, 105.0, 525.0, 105.0, 495.0, 43.346157073974609, 495.0 ],
					"source" : [ "obj-165", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-236", 0 ],
					"midpoints" : [ 451.525311708450317, 525.0, 426.166656732559204, 525.0 ],
					"source" : [ "obj-165", 2 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-165", 4 ],
					"midpoints" : [ 423.424045205116272, 486.0, 451.525311708450317, 486.0 ],
					"source" : [ "obj-166", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-41", 0 ],
					"midpoints" : [ 328.0, 36.0, 329.0, 36.0 ],
					"source" : [ "obj-167", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-67", 0 ],
					"midpoints" : [ 724.0, 36.0, 725.0, 36.0 ],
					"source" : [ "obj-168", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-56", 0 ],
					"midpoints" : [ 643.0, 36.0, 644.0, 36.0 ],
					"source" : [ "obj-169", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-19", 0 ],
					"midpoints" : [ 233.228399991989136, 756.0, 273.0, 756.0, 273.0, 729.0, 296.191367983818054, 729.0 ],
					"source" : [ "obj-17", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-52", 0 ],
					"midpoints" : [ 563.0, 36.0, 564.0, 36.0 ],
					"source" : [ "obj-170", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-118", 0 ],
					"midpoints" : [ 511.08730936050415, 1234.884467840194702, 555.976198077201843, 1234.884467840194702, 555.976198077201843, 1207.884467840194702, 569.817469000816345, 1207.884467840194702 ],
					"order" : 1,
					"source" : [ "obj-171", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-210", 0 ],
					"midpoints" : [ 511.08730936050415, 1234.884467840194702, 657.976198077201843, 1234.884467840194702, 657.976198077201843, 1216.884467840194702, 682.515883445739746, 1216.884467840194702 ],
					"order" : 0,
					"source" : [ "obj-171", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-214", 0 ],
					"midpoints" : [ 39.671716332435608, 297.0, 15.0, 297.0, 15.0, 351.0, 39.671716332435608, 351.0 ],
					"source" : [ "obj-172", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-171", 0 ],
					"midpoints" : [ 562.26191258430481, 1204.884467840194702, 511.08730936050415, 1204.884467840194702 ],
					"source" : [ "obj-174", 2 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-181", 0 ],
					"midpoints" : [ 185.5, 330.0, 185.5, 330.0 ],
					"source" : [ "obj-175", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-181", 3 ],
					"midpoints" : [ 239.5, 300.0, 225.0, 300.0, 225.0, 330.0, 279.25, 330.0 ],
					"source" : [ "obj-176", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-181", 2 ],
					"midpoints" : [ 215.5, 303.0, 248.0, 303.0 ],
					"source" : [ "obj-178", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-181", 1 ],
					"midpoints" : [ 185.5, 300.0, 171.0, 300.0, 171.0, 330.0, 216.75, 330.0 ],
					"source" : [ "obj-179", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-17", 0 ],
					"midpoints" : [ 233.228399991989136, 717.0, 233.228399991989136, 717.0 ],
					"source" : [ "obj-18", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-2", 0 ],
					"midpoints" : [ 185.5, 372.0, 153.0, 372.0, 153.0, 495.0, 43.346157073974609, 495.0 ],
					"source" : [ "obj-181", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-230", 0 ],
					"source" : [ "obj-181", 2 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-240", 0 ],
					"source" : [ "obj-181", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-190", 0 ],
					"midpoints" : [ 742.411382794380188, 483.0, 742.411382794380188, 483.0 ],
					"source" : [ "obj-183", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-190", 3 ],
					"midpoints" : [ 800.639230132102966, 450.0, 783.0, 450.0, 783.0, 483.0, 806.911382794380188, 483.0 ],
					"source" : [ "obj-184", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-232", 8 ],
					"midpoints" : [ 800.639230132102966, 558.0, 617.720610618591309, 558.0 ],
					"source" : [ "obj-185", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-190", 2 ],
					"midpoints" : [ 771.525306463241577, 483.0, 785.411382794380188, 483.0 ],
					"source" : [ "obj-186", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-190", 1 ],
					"midpoints" : [ 743.677205562591553, 453.0, 729.0, 453.0, 729.0, 483.0, 763.911382794380188, 483.0 ],
					"source" : [ "obj-187", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-6", 0 ],
					"midpoints" : [ 296.191367983818054, 759.0, 290.709903955459595, 759.0 ],
					"source" : [ "obj-19", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-185", 0 ],
					"midpoints" : [ 828.411382794380188, 525.0, 837.0, 525.0, 837.0, 522.0, 852.0, 522.0, 852.0, 558.0, 786.0, 558.0, 786.0, 525.0, 800.639230132102966, 525.0 ],
					"source" : [ "obj-190", 2 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-188", 0 ],
					"midpoints" : [ 785.411382794380188, 558.0, 729.0, 558.0, 729.0, 522.0, 742.411382794380188, 522.0 ],
					"source" : [ "obj-190", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-2", 0 ],
					"midpoints" : [ 742.411382794380188, 525.0, 714.0, 525.0, 714.0, 411.0, 153.0, 411.0, 153.0, 495.0, 43.346157073974609, 495.0 ],
					"source" : [ "obj-190", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-190", 4 ],
					"midpoints" : [ 795.575939059257507, 483.0, 828.411382794380188, 483.0 ],
					"source" : [ "obj-191", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-198", 0 ],
					"midpoints" : [ 618.360751509666443, 483.0, 618.360751509666443, 483.0 ],
					"source" : [ "obj-192", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-198", 3 ],
					"midpoints" : [ 674.056953310966492, 456.0, 657.0, 456.0, 657.0, 483.0, 682.860751509666443, 483.0 ],
					"source" : [ "obj-193", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-232", 7 ],
					"midpoints" : [ 671.525307774543762, 558.0, 596.756324708461761, 558.0 ],
					"source" : [ "obj-194", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-198", 2 ],
					"midpoints" : [ 647.474675178527832, 453.0, 661.360751509666443, 453.0 ],
					"source" : [ "obj-195", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-198", 1 ],
					"midpoints" : [ 623.424042582511902, 453.0, 648.0, 453.0, 648.0, 483.0, 639.860751509666443, 483.0 ],
					"source" : [ "obj-196", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-194", 0 ],
					"midpoints" : [ 704.360751509666443, 525.0, 671.525307774543762, 525.0 ],
					"source" : [ "obj-198", 2 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-197", 0 ],
					"midpoints" : [ 661.360751509666443, 549.0, 603.0, 549.0, 603.0, 522.0, 618.360751509666443, 522.0 ],
					"source" : [ "obj-198", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-2", 0 ],
					"midpoints" : [ 618.360751509666443, 525.0, 591.0, 525.0, 591.0, 411.0, 153.0, 411.0, 153.0, 495.0, 43.346157073974609, 495.0 ],
					"source" : [ "obj-198", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-198", 4 ],
					"midpoints" : [ 671.525307774543762, 483.0, 704.360751509666443, 483.0 ],
					"source" : [ "obj-199", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-22", 0 ],
					"midpoints" : [ 43.346157073974609, 621.0, 43.346157073974609, 621.0 ],
					"source" : [ "obj-2", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-1", 0 ],
					"midpoints" : [ 43.346157073974609, 687.0, 30.0, 687.0, 30.0, 714.0, 70.846160888671875, 714.0 ],
					"order" : 1,
					"source" : [ "obj-20", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-29", 0 ],
					"midpoints" : [ 43.346157073974609, 681.0, 43.346157073974609, 681.0 ],
					"order" : 2,
					"source" : [ "obj-20", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-6", 0 ],
					"midpoints" : [ 43.346157073974609, 681.0, 18.0, 681.0, 18.0, 762.0, 290.709903955459595, 762.0 ],
					"order" : 0,
					"source" : [ "obj-20", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-206", 0 ],
					"midpoints" : [ 490.512651920318604, 483.0, 490.512651920318604, 483.0 ],
					"source" : [ "obj-200", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-206", 3 ],
					"midpoints" : [ 547.474676489830017, 450.0, 531.0, 450.0, 531.0, 483.0, 555.012651920318604, 483.0 ],
					"source" : [ "obj-201", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-232", 6 ],
					"midpoints" : [ 547.474676489830017, 558.0, 575.792038798332214, 558.0 ],
					"source" : [ "obj-202", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-206", 2 ],
					"midpoints" : [ 520.892398357391357, 456.0, 533.512651920318604, 456.0 ],
					"source" : [ "obj-203", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-206", 1 ],
					"midpoints" : [ 490.512651920318604, 453.0, 477.0, 453.0, 477.0, 483.0, 512.012651920318604, 483.0 ],
					"source" : [ "obj-204", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-2", 0 ],
					"midpoints" : [ 490.512651920318604, 525.0, 105.0, 525.0, 105.0, 495.0, 43.346157073974609, 495.0 ],
					"source" : [ "obj-206", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-202", 0 ],
					"midpoints" : [ 576.512651920318604, 525.0, 588.0, 525.0, 588.0, 549.0, 534.0, 549.0, 534.0, 525.0, 547.474676489830017, 525.0 ],
					"source" : [ "obj-206", 2 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-205", 0 ],
					"midpoints" : [ 533.512651920318604, 549.0, 477.0, 549.0, 477.0, 522.0, 490.512651920318604, 522.0 ],
					"source" : [ "obj-206", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-206", 4 ],
					"midpoints" : [ 543.677208185195923, 483.0, 576.512651920318604, 483.0 ],
					"source" : [ "obj-207", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-174", 0 ],
					"midpoints" : [ 514.26191258430481, 1177.884467840194702, 514.26191258430481, 1177.884467840194702 ],
					"source" : [ "obj-208", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-214", 1 ],
					"midpoints" : [ 88.156562447547913, 345.0, 57.0, 345.0, 57.0, 351.0, 55.171716332435608, 351.0 ],
					"source" : [ "obj-209", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-212", 0 ],
					"midpoints" : [ 769.817472100257874, 1243.884467840194702, 516.976198077201843, 1243.884467840194702, 516.976198077201843, 1249.884467840194702, 514.26191258430481, 1249.884467840194702 ],
					"source" : [ "obj-21", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-212", 0 ],
					"midpoints" : [ 682.515883445739746, 1243.884467840194702, 516.976198077201843, 1243.884467840194702, 516.976198077201843, 1249.884467840194702, 514.26191258430481, 1249.884467840194702 ],
					"source" : [ "obj-210", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-28", 0 ],
					"midpoints" : [ 192.695649147033691, 1044.0, 168.0, 1044.0, 168.0, 1005.0, 262.260865211486816, 1005.0 ],
					"source" : [ "obj-211", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-126", 0 ],
					"midpoints" : [ 817.26191258430481, 1435.884467840194702, 828.976198077201843, 1435.884467840194702, 828.976198077201843, 1468.884467840194702, 514.26191258430481, 1468.884467840194702 ],
					"source" : [ "obj-212", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-2", 0 ],
					"midpoints" : [ 39.671716332435608, 381.0, 15.0, 381.0, 15.0, 582.0, 43.346157073974609, 582.0 ],
					"source" : [ "obj-214", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-229", 0 ],
					"source" : [ "obj-214", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-214", 2 ],
					"midpoints" : [ 104.0, 420.0, 81.0, 420.0, 81.0, 351.0, 70.671716332435608, 351.0 ],
					"source" : [ "obj-215", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-214", 0 ],
					"midpoints" : [ 36.546263337135315, 345.0, 39.671716332435608, 345.0 ],
					"source" : [ "obj-216", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-28", 0 ],
					"midpoints" : [ 225.304344177246094, 1044.0, 249.0, 1044.0, 249.0, 1014.0, 262.260865211486816, 1014.0 ],
					"source" : [ "obj-217", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-20", 0 ],
					"midpoints" : [ 43.346157073974609, 651.0, 43.346157073974609, 651.0 ],
					"order" : 1,
					"source" : [ "obj-22", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-220", 0 ],
					"order" : 0,
					"source" : [ "obj-22", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-218", 0 ],
					"source" : [ "obj-229", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-212", 0 ],
					"midpoints" : [ 682.515883445739746, 1216.884467840194702, 612.976198077201843, 1216.884467840194702, 612.976198077201843, 1249.884467840194702, 514.26191258430481, 1249.884467840194702 ],
					"source" : [ "obj-23", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-232", 0 ],
					"source" : [ "obj-230", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-232", 1 ],
					"midpoints" : [ 424.424045205116272, 414.0, 470.970609247684479, 414.0 ],
					"source" : [ "obj-231", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-228", 0 ],
					"midpoints" : [ 450.006323337554932, 588.0, 408.0, 588.0, 408.0, 558.0, 241.145566582679749, 558.0 ],
					"source" : [ "obj-232", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-232", 2 ],
					"midpoints" : [ 543.411385416984558, 411.0, 468.0, 411.0, 468.0, 558.0, 491.934895157814026, 558.0 ],
					"source" : [ "obj-233", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-232", 3 ],
					"midpoints" : [ 669.025307774543762, 411.0, 591.0, 411.0, 591.0, 558.0, 512.899181067943573, 558.0 ],
					"source" : [ "obj-234", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-232", 4 ],
					"midpoints" : [ 805.33331435918808, 411.0, 591.0, 411.0, 591.0, 555.0, 533.86346697807312, 555.0 ],
					"source" : [ "obj-235", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-232", 5 ],
					"midpoints" : [ 426.166656732559204, 558.0, 554.827752888202667, 558.0 ],
					"source" : [ "obj-236", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-239", 0 ],
					"order" : 1,
					"source" : [ "obj-244", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-245", 0 ],
					"order" : 0,
					"source" : [ "obj-244", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-245", 0 ],
					"source" : [ "obj-246", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-246", 0 ],
					"source" : [ "obj-248", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-77", 0 ],
					"midpoints" : [ 49.217391014099121, 996.0, 49.217391014099121, 996.0 ],
					"source" : [ "obj-25", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-14", 0 ],
					"midpoints" : [ 49.217391014099121, 1188.0, 49.217391014099121, 1188.0 ],
					"source" : [ "obj-26", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-282", 0 ],
					"midpoints" : [ 1776.340412855148315, 38.16743803024292, 1777.340412855148315, 38.16743803024292 ],
					"source" : [ "obj-264", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-283", 0 ],
					"midpoints" : [ 1695.489349603652954, 38.16743803024292, 1696.489349603652954, 38.16743803024292 ],
					"source" : [ "obj-265", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-284", 0 ],
					"midpoints" : [ 1614.638286352157593, 38.16743803024292, 1615.638286352157593, 38.16743803024292 ],
					"source" : [ "obj-266", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-285", 0 ],
					"midpoints" : [ 1538.042542219161987, 38.16743803024292, 1539.042542219161987, 38.16743803024292 ],
					"source" : [ "obj-267", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-286", 0 ],
					"midpoints" : [ 1461.446798086166382, 38.16743803024292, 1462.446798086166382, 38.16743803024292 ],
					"source" : [ "obj-268", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-287", 0 ],
					"midpoints" : [ 1380.595734834671021, 38.16743803024292, 1381.595734834671021, 38.16743803024292 ],
					"source" : [ "obj-269", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-23", 0 ],
					"midpoints" : [ 682.515883445739746, 1189.884467840194702, 682.515883445739746, 1189.884467840194702 ],
					"source" : [ "obj-27", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-288", 0 ],
					"midpoints" : [ 1299.744671583175659, 38.16743803024292, 1300.744671583175659, 38.16743803024292 ],
					"source" : [ "obj-270", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-289", 0 ],
					"midpoints" : [ 1225.276587009429932, 38.16743803024292, 1226.276587009429932, 38.16743803024292 ],
					"source" : [ "obj-271", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-290", 0 ],
					"midpoints" : [ 1144.42552375793457, 32.16743803024292, 1145.42552375793457, 32.16743803024292 ],
					"source" : [ "obj-272", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-24", 0 ],
					"midpoints" : [ 262.260865211486816, 1044.0, 180.0, 1044.0, 180.0, 1077.0, 188.804344177246094, 1077.0 ],
					"source" : [ "obj-28", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-292", 1 ],
					"midpoints" : [ 1777.340412855148315, 104.16743803024292, 1754.23683762550354, 104.16743803024292, 1754.23683762550354, 158.16743803024292, 1795.712753295898438, 158.16743803024292 ],
					"source" : [ "obj-282", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-294", 1 ],
					"midpoints" : [ 1696.489349603652954, 104.16743803024292, 1676.23683762550354, 104.16743803024292, 1676.23683762550354, 158.16743803024292, 1716.989349603652954, 158.16743803024292 ],
					"source" : [ "obj-283", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-296", 1 ],
					"midpoints" : [ 1615.638286352157593, 104.16743803024292, 1595.23683762550354, 104.16743803024292, 1595.23683762550354, 158.16743803024292, 1638.265945911407471, 158.16743803024292 ],
					"source" : [ "obj-284", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-298", 1 ],
					"midpoints" : [ 1539.042542219161987, 104.16743803024292, 1517.23683762550354, 104.16743803024292, 1517.23683762550354, 158.16743803024292, 1559.542542219161987, 158.16743803024292 ],
					"source" : [ "obj-285", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-300", 1 ],
					"midpoints" : [ 1462.446798086166382, 104.16743803024292, 1442.23683762550354, 104.16743803024292, 1442.23683762550354, 158.16743803024292, 1482.946798086166382, 158.16743803024292 ],
					"source" : [ "obj-286", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-302", 1 ],
					"midpoints" : [ 1381.595734834671021, 104.16743803024292, 1361.23683762550354, 104.16743803024292, 1361.23683762550354, 158.16743803024292, 1402.095734834671021, 158.16743803024292 ],
					"source" : [ "obj-287", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-304", 1 ],
					"midpoints" : [ 1300.744671583175659, 104.16743803024292, 1280.23683762550354, 104.16743803024292, 1280.23683762550354, 158.16743803024292, 1321.244671583175659, 158.16743803024292 ],
					"source" : [ "obj-288", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-306", 1 ],
					"midpoints" : [ 1226.276587009429932, 104.16743803024292, 1205.23683762550354, 104.16743803024292, 1205.23683762550354, 158.16743803024292, 1246.776587009429932, 158.16743803024292 ],
					"source" : [ "obj-289", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-317", 1 ],
					"midpoints" : [ 1145.42552375793457, 104.16743803024292, 1124.23683762550354, 104.16743803024292, 1124.23683762550354, 158.16743803024292, 1165.92552375793457, 158.16743803024292 ],
					"source" : [ "obj-290", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-264", 0 ],
					"midpoints" : [ 1779.712753295898438, 158.16743803024292, 1754.23683762550354, 158.16743803024292, 1754.23683762550354, 20.16743803024292, 1776.340412855148315, 20.16743803024292 ],
					"order" : 0,
					"source" : [ "obj-291", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-292", 0 ],
					"midpoints" : [ 1779.712753295898438, 158.16743803024292, 1779.712753295898438, 158.16743803024292 ],
					"order" : 1,
					"source" : [ "obj-291", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-273", 0 ],
					"midpoints" : [ 1779.712753295898438, 185.16743803024292, 1754.23683762550354, 185.16743803024292, 1754.23683762550354, 35.16743803024292, 1794.606370210647583, 35.16743803024292 ],
					"order" : 0,
					"source" : [ "obj-292", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-318", 0 ],
					"midpoints" : [ 1779.712753295898438, 185.16743803024292, 1779.712753295898438, 185.16743803024292 ],
					"order" : 1,
					"source" : [ "obj-292", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-265", 0 ],
					"midpoints" : [ 1700.989349603652954, 158.16743803024292, 1676.23683762550354, 158.16743803024292, 1676.23683762550354, 20.16743803024292, 1695.489349603652954, 20.16743803024292 ],
					"order" : 1,
					"source" : [ "obj-293", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-294", 0 ],
					"midpoints" : [ 1700.989349603652954, 158.16743803024292, 1700.989349603652954, 158.16743803024292 ],
					"order" : 0,
					"source" : [ "obj-293", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-274", 0 ],
					"midpoints" : [ 1700.989349603652954, 185.16743803024292, 1676.23683762550354, 185.16743803024292, 1676.23683762550354, 35.16743803024292, 1715.8829665184021, 35.16743803024292 ],
					"order" : 0,
					"source" : [ "obj-294", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-319", 0 ],
					"midpoints" : [ 1700.989349603652954, 185.16743803024292, 1700.989349603652954, 185.16743803024292 ],
					"order" : 1,
					"source" : [ "obj-294", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-266", 0 ],
					"midpoints" : [ 1622.265945911407471, 158.16743803024292, 1595.23683762550354, 158.16743803024292, 1595.23683762550354, 20.16743803024292, 1614.638286352157593, 20.16743803024292 ],
					"order" : 1,
					"source" : [ "obj-295", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-296", 0 ],
					"midpoints" : [ 1622.265945911407471, 158.16743803024292, 1622.265945911407471, 158.16743803024292 ],
					"order" : 0,
					"source" : [ "obj-295", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-275", 0 ],
					"midpoints" : [ 1622.265945911407471, 185.16743803024292, 1598.23683762550354, 185.16743803024292, 1598.23683762550354, 35.16743803024292, 1641.414881944656372, 35.16743803024292 ],
					"order" : 0,
					"source" : [ "obj-296", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-320", 0 ],
					"midpoints" : [ 1622.265945911407471, 185.16743803024292, 1622.265945911407471, 185.16743803024292 ],
					"order" : 1,
					"source" : [ "obj-296", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-267", 0 ],
					"midpoints" : [ 1543.542542219161987, 158.16743803024292, 1517.23683762550354, 158.16743803024292, 1517.23683762550354, 20.16743803024292, 1538.042542219161987, 20.16743803024292 ],
					"order" : 1,
					"source" : [ "obj-297", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-298", 0 ],
					"midpoints" : [ 1543.542542219161987, 158.16743803024292, 1543.542542219161987, 158.16743803024292 ],
					"order" : 0,
					"source" : [ "obj-297", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-276", 0 ],
					"midpoints" : [ 1543.542542219161987, 185.16743803024292, 1517.23683762550354, 185.16743803024292, 1517.23683762550354, 35.16743803024292, 1560.563818693161011, 35.16743803024292 ],
					"order" : 0,
					"source" : [ "obj-298", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-321", 0 ],
					"midpoints" : [ 1543.542542219161987, 185.16743803024292, 1543.542542219161987, 185.16743803024292 ],
					"order" : 1,
					"source" : [ "obj-298", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-268", 0 ],
					"midpoints" : [ 1466.946798086166382, 158.16743803024292, 1442.23683762550354, 158.16743803024292, 1442.23683762550354, 20.16743803024292, 1461.446798086166382, 20.16743803024292 ],
					"order" : 1,
					"source" : [ "obj-299", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-300", 0 ],
					"midpoints" : [ 1466.946798086166382, 158.16743803024292, 1466.946798086166382, 158.16743803024292 ],
					"order" : 0,
					"source" : [ "obj-299", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-84", 1 ],
					"midpoints" : [ 92.0, 105.0, 72.0, 105.0, 72.0, 159.0, 112.5, 159.0 ],
					"source" : [ "obj-3", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-32", 0 ],
					"midpoints" : [ 216.608692169189453, 1254.0, 51.0, 1254.0, 51.0, 1260.0, 49.217391014099121, 1260.0 ],
					"source" : [ "obj-30", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-277", 0 ],
					"midpoints" : [ 1466.946798086166382, 185.16743803024292, 1442.23683762550354, 185.16743803024292, 1442.23683762550354, 35.16743803024292, 1481.840415000915527, 35.16743803024292 ],
					"order" : 0,
					"source" : [ "obj-300", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-322", 0 ],
					"midpoints" : [ 1466.946798086166382, 185.16743803024292, 1466.946798086166382, 185.16743803024292 ],
					"order" : 1,
					"source" : [ "obj-300", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-269", 0 ],
					"midpoints" : [ 1386.095734834671021, 158.16743803024292, 1361.23683762550354, 158.16743803024292, 1361.23683762550354, 20.16743803024292, 1380.595734834671021, 20.16743803024292 ],
					"order" : 1,
					"source" : [ "obj-301", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-302", 0 ],
					"midpoints" : [ 1386.095734834671021, 158.16743803024292, 1386.095734834671021, 158.16743803024292 ],
					"order" : 0,
					"source" : [ "obj-301", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-278", 0 ],
					"midpoints" : [ 1386.095734834671021, 185.16743803024292, 1361.23683762550354, 185.16743803024292, 1361.23683762550354, 35.16743803024292, 1400.989351749420166, 35.16743803024292 ],
					"order" : 0,
					"source" : [ "obj-302", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-323", 0 ],
					"midpoints" : [ 1386.095734834671021, 185.16743803024292, 1386.095734834671021, 185.16743803024292 ],
					"order" : 1,
					"source" : [ "obj-302", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-270", 0 ],
					"midpoints" : [ 1305.244671583175659, 158.16743803024292, 1280.23683762550354, 158.16743803024292, 1280.23683762550354, 20.16743803024292, 1299.744671583175659, 20.16743803024292 ],
					"order" : 1,
					"source" : [ "obj-303", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-304", 0 ],
					"midpoints" : [ 1305.244671583175659, 158.16743803024292, 1305.244671583175659, 158.16743803024292 ],
					"order" : 0,
					"source" : [ "obj-303", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-279", 0 ],
					"midpoints" : [ 1305.244671583175659, 185.16743803024292, 1280.23683762550354, 185.16743803024292, 1280.23683762550354, 35.16743803024292, 1320.138288497924805, 35.16743803024292 ],
					"order" : 0,
					"source" : [ "obj-304", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-324", 0 ],
					"midpoints" : [ 1305.244671583175659, 185.16743803024292, 1305.244671583175659, 185.16743803024292 ],
					"order" : 1,
					"source" : [ "obj-304", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-271", 0 ],
					"midpoints" : [ 1230.776587009429932, 158.16743803024292, 1205.23683762550354, 158.16743803024292, 1205.23683762550354, 20.16743803024292, 1225.276587009429932, 20.16743803024292 ],
					"order" : 1,
					"source" : [ "obj-305", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-306", 0 ],
					"midpoints" : [ 1230.776587009429932, 158.16743803024292, 1230.776587009429932, 158.16743803024292 ],
					"order" : 0,
					"source" : [ "obj-305", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-280", 0 ],
					"midpoints" : [ 1230.776587009429932, 185.16743803024292, 1205.23683762550354, 185.16743803024292, 1205.23683762550354, 35.16743803024292, 1243.542544364929199, 35.16743803024292 ],
					"order" : 0,
					"source" : [ "obj-306", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-325", 0 ],
					"midpoints" : [ 1230.776587009429932, 185.16743803024292, 1230.776587009429932, 185.16743803024292 ],
					"order" : 1,
					"source" : [ "obj-306", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-292", 2 ],
					"midpoints" : [ 1781.840412855148315, 134.16743803024292, 1766.23683762550354, 134.16743803024292, 1766.23683762550354, 158.16743803024292, 1811.712753295898438, 158.16743803024292 ],
					"source" : [ "obj-307", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-294", 2 ],
					"midpoints" : [ 1700.989349603652954, 134.16743803024292, 1688.23683762550354, 134.16743803024292, 1688.23683762550354, 155.16743803024292, 1732.989349603652954, 155.16743803024292 ],
					"source" : [ "obj-308", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-296", 2 ],
					"midpoints" : [ 1622.265945911407471, 134.16743803024292, 1610.23683762550354, 134.16743803024292, 1610.23683762550354, 155.16743803024292, 1654.265945911407471, 155.16743803024292 ],
					"source" : [ "obj-309", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-298", 2 ],
					"midpoints" : [ 1543.542542219161987, 134.16743803024292, 1529.23683762550354, 134.16743803024292, 1529.23683762550354, 158.16743803024292, 1575.542542219161987, 158.16743803024292 ],
					"source" : [ "obj-310", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-300", 2 ],
					"midpoints" : [ 1466.946798086166382, 134.16743803024292, 1451.23683762550354, 134.16743803024292, 1451.23683762550354, 158.16743803024292, 1498.946798086166382, 158.16743803024292 ],
					"source" : [ "obj-311", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-302", 2 ],
					"midpoints" : [ 1386.095734834671021, 134.16743803024292, 1373.23683762550354, 134.16743803024292, 1373.23683762550354, 155.16743803024292, 1418.095734834671021, 155.16743803024292 ],
					"source" : [ "obj-312", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-304", 2 ],
					"midpoints" : [ 1305.244671583175659, 134.16743803024292, 1292.23683762550354, 134.16743803024292, 1292.23683762550354, 158.16743803024292, 1337.244671583175659, 158.16743803024292 ],
					"source" : [ "obj-313", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-306", 2 ],
					"midpoints" : [ 1230.776587009429932, 134.16743803024292, 1214.23683762550354, 134.16743803024292, 1214.23683762550354, 158.16743803024292, 1262.776587009429932, 158.16743803024292 ],
					"source" : [ "obj-314", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-317", 2 ],
					"midpoints" : [ 1149.92552375793457, 131.16743803024292, 1136.23683762550354, 131.16743803024292, 1136.23683762550354, 158.16743803024292, 1181.92552375793457, 158.16743803024292 ],
					"source" : [ "obj-315", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-272", 0 ],
					"midpoints" : [ 1149.92552375793457, 158.16743803024292, 1124.23683762550354, 158.16743803024292, 1124.23683762550354, 17.16743803024292, 1144.42552375793457, 17.16743803024292 ],
					"order" : 1,
					"source" : [ "obj-316", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-317", 0 ],
					"midpoints" : [ 1149.92552375793457, 158.16743803024292, 1149.92552375793457, 158.16743803024292 ],
					"order" : 0,
					"source" : [ "obj-316", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-281", 0 ],
					"midpoints" : [ 1149.92552375793457, 185.16743803024292, 1124.23683762550354, 185.16743803024292, 1124.23683762550354, 35.16743803024292, 1166.946800231933594, 35.16743803024292 ],
					"order" : 0,
					"source" : [ "obj-317", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-326", 0 ],
					"midpoints" : [ 1149.92552375793457, 185.16743803024292, 1149.92552375793457, 185.16743803024292 ],
					"order" : 1,
					"source" : [ "obj-317", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-36", 0 ],
					"midpoints" : [ 352.217391014099121, 1446.0, 363.0, 1446.0, 363.0, 1479.0, 49.217391014099121, 1479.0 ],
					"source" : [ "obj-32", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-333", 0 ],
					"midpoints" : [ 1053.505219340324402, 330.0, 1053.505219340324402, 330.0 ],
					"source" : [ "obj-327", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-333", 3 ],
					"midpoints" : [ 1106.929872989654541, 300.0, 1092.0, 300.0, 1092.0, 330.0, 1113.505219340324402, 330.0 ],
					"source" : [ "obj-328", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-5", 0 ],
					"source" : [ "obj-329", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-30", 0 ],
					"midpoints" : [ 44.869565010070801, 1245.0, 192.0, 1245.0, 192.0, 1227.0, 216.608692169189453, 1227.0 ],
					"order" : 0,
					"source" : [ "obj-33", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-76", 0 ],
					"midpoints" : [ 44.869565010070801, 1245.0, 90.0, 1245.0, 90.0, 1218.0, 103.565216064453125, 1218.0 ],
					"order" : 1,
					"source" : [ "obj-33", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-333", 2 ],
					"midpoints" : [ 1082.272340536117554, 300.0, 1092.0, 300.0, 1092.0, 330.0, 1093.505219340324402, 330.0 ],
					"source" : [ "obj-330", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-333", 1 ],
					"midpoints" : [ 1053.505219340324402, 300.0, 1041.0, 300.0, 1041.0, 330.0, 1073.505219340324402, 330.0 ],
					"source" : [ "obj-331", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-329", 0 ],
					"midpoints" : [ 1133.505219340324402, 369.0, 1106.929872989654541, 369.0 ],
					"source" : [ "obj-333", 2 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-332", 0 ],
					"midpoints" : [ 1093.505219340324402, 369.0, 1053.505219340324402, 369.0 ],
					"source" : [ "obj-333", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-425", 0 ],
					"midpoints" : [ 1053.505219340324402, 369.0, 1041.0, 369.0, 1041.0, 603.0, 1228.418837547302246, 603.0 ],
					"source" : [ "obj-333", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-333", 4 ],
					"midpoints" : [ 1106.929872989654541, 330.0, 1133.505219340324402, 330.0 ],
					"source" : [ "obj-334", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-341", 0 ],
					"midpoints" : [ 1609.669562458992004, 483.0, 1609.669562458992004, 483.0 ],
					"source" : [ "obj-335", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-341", 3 ],
					"midpoints" : [ 1668.573667764663696, 450.0, 1650.0, 450.0, 1650.0, 483.0, 1674.169562458992004, 483.0 ],
					"source" : [ "obj-336", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-5", 8 ],
					"source" : [ "obj-337", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-341", 2 ],
					"midpoints" : [ 1638.436683654785156, 453.0, 1647.0, 453.0, 1647.0, 483.0, 1652.669562458992004, 483.0 ],
					"source" : [ "obj-338", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-341", 1 ],
					"midpoints" : [ 1611.039425373077393, 453.0, 1596.0, 453.0, 1596.0, 483.0, 1631.169562458992004, 483.0 ],
					"source" : [ "obj-339", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-18", 1 ],
					"midpoints" : [ 247.228399991989136, 684.0, 269.228399991989136, 684.0 ],
					"source" : [ "obj-34", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-337", 0 ],
					"midpoints" : [ 1695.669562458992004, 525.0, 1668.573667764663696, 525.0 ],
					"source" : [ "obj-341", 2 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-340", 0 ],
					"midpoints" : [ 1652.669562458992004, 558.0, 1596.0, 558.0, 1596.0, 522.0, 1609.669562458992004, 522.0 ],
					"source" : [ "obj-341", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-425", 0 ],
					"source" : [ "obj-341", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-341", 4 ],
					"midpoints" : [ 1663.094216108322144, 483.0, 1695.669562458992004, 483.0 ],
					"source" : [ "obj-342", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-349", 0 ],
					"midpoints" : [ 1486.381900191307068, 483.0, 1486.381900191307068, 483.0 ],
					"source" : [ "obj-343", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-349", 3 ],
					"midpoints" : [ 1541.176416754722595, 456.0, 1524.0, 456.0, 1524.0, 483.0, 1550.881900191307068, 483.0 ],
					"source" : [ "obj-344", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-5", 7 ],
					"source" : [ "obj-345", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-349", 2 ],
					"midpoints" : [ 1515.14902138710022, 453.0, 1524.0, 453.0, 1524.0, 483.0, 1529.381900191307068, 483.0 ],
					"source" : [ "obj-346", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-349", 1 ],
					"midpoints" : [ 1490.491488933563232, 453.0, 1473.0, 453.0, 1473.0, 483.0, 1507.881900191307068, 483.0 ],
					"source" : [ "obj-347", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-345", 0 ],
					"midpoints" : [ 1572.381900191307068, 525.0, 1581.0, 525.0, 1581.0, 558.0, 1538.436690926551819, 558.0 ],
					"source" : [ "obj-349", 2 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-348", 0 ],
					"midpoints" : [ 1529.381900191307068, 558.0, 1473.0, 558.0, 1473.0, 522.0, 1486.381900191307068, 522.0 ],
					"source" : [ "obj-349", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-425", 0 ],
					"source" : [ "obj-349", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-32", 0 ],
					"midpoints" : [ 303.565212249755859, 1254.0, 51.0, 1254.0, 51.0, 1260.0, 49.217391014099121, 1260.0 ],
					"source" : [ "obj-35", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-349", 4 ],
					"midpoints" : [ 1538.436690926551819, 483.0, 1572.381900191307068, 483.0 ],
					"source" : [ "obj-350", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-357", 0 ],
					"midpoints" : [ 1357.614786267280579, 483.0, 1357.614786267280579, 483.0 ],
					"source" : [ "obj-351", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-357", 3 ],
					"midpoints" : [ 1415.149028658866882, 450.0, 1398.0, 450.0, 1398.0, 483.0, 1422.114786267280579, 483.0 ],
					"source" : [ "obj-352", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-5", 6 ],
					"source" : [ "obj-353", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-357", 2 ],
					"midpoints" : [ 1387.751770377159119, 453.0, 1395.0, 453.0, 1395.0, 483.0, 1400.614786267280579, 483.0 ],
					"source" : [ "obj-354", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-357", 1 ],
					"midpoints" : [ 1357.614786267280579, 453.0, 1344.0, 453.0, 1344.0, 483.0, 1379.114786267280579, 483.0 ],
					"source" : [ "obj-355", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-353", 0 ],
					"midpoints" : [ 1443.614786267280579, 525.0, 1415.149028658866882, 525.0 ],
					"source" : [ "obj-357", 2 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-356", 0 ],
					"midpoints" : [ 1400.614786267280579, 558.0, 1344.0, 558.0, 1344.0, 522.0, 1357.614786267280579, 522.0 ],
					"source" : [ "obj-357", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-425", 0 ],
					"source" : [ "obj-357", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-357", 4 ],
					"midpoints" : [ 1411.039439916610718, 483.0, 1443.614786267280579, 483.0 ],
					"source" : [ "obj-358", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-365", 0 ],
					"midpoints" : [ 1238.436712741851807, 486.0, 1238.436712741851807, 486.0 ],
					"source" : [ "obj-359", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-86", 0 ],
					"midpoints" : [ 49.217391014099121, 1506.0, 40.52173900604248, 1506.0 ],
					"source" : [ "obj-36", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-91", 0 ],
					"midpoints" : [ 92.217391014099121, 1506.0, 77.478260040283203, 1506.0 ],
					"source" : [ "obj-36", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-92", 0 ],
					"midpoints" : [ 135.217391014099121, 1506.0, 120.956520080566406, 1506.0 ],
					"source" : [ "obj-36", 2 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-93", 0 ],
					"midpoints" : [ 178.217391014099121, 1506.0, 157.913041114807129, 1506.0 ],
					"source" : [ "obj-36", 3 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-94", 0 ],
					"midpoints" : [ 221.217391014099121, 1506.0, 205.739127159118652, 1506.0 ],
					"source" : [ "obj-36", 4 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-95", 0 ],
					"midpoints" : [ 264.217391014099121, 1506.0, 257.913039207458496, 1506.0 ],
					"source" : [ "obj-36", 5 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-96", 0 ],
					"midpoints" : [ 307.217391014099121, 1506.0, 303.565212249755859, 1506.0 ],
					"source" : [ "obj-36", 6 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-97", 0 ],
					"midpoints" : [ 350.217391014099121, 1506.0, 349.217385292053223, 1506.0 ],
					"source" : [ "obj-36", 7 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-98", 0 ],
					"midpoints" : [ 393.217391014099121, 1506.0, 392.695645332336426, 1506.0 ],
					"source" : [ "obj-36", 8 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-365", 3 ],
					"midpoints" : [ 1294.601092219352722, 453.0, 1275.0, 453.0, 1275.0, 486.0, 1298.436712741851807, 486.0 ],
					"source" : [ "obj-360", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-5", 5 ],
					"source" : [ "obj-361", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-365", 2 ],
					"midpoints" : [ 1268.573696851730347, 456.0, 1275.0, 456.0, 1275.0, 486.0, 1278.436712741851807, 486.0 ],
					"source" : [ "obj-362", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-365", 1 ],
					"midpoints" : [ 1238.436712741851807, 456.0, 1224.0, 456.0, 1224.0, 486.0, 1258.436712741851807, 486.0 ],
					"source" : [ "obj-363", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-361", 0 ],
					"midpoints" : [ 1318.436712741851807, 525.0, 1294.601092219352722, 525.0 ],
					"source" : [ "obj-365", 2 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-364", 0 ],
					"midpoints" : [ 1278.436712741851807, 525.0, 1238.436712741851807, 525.0 ],
					"source" : [ "obj-365", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-425", 0 ],
					"source" : [ "obj-365", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-365", 4 ],
					"midpoints" : [ 1290.491503477096558, 486.0, 1318.436712741851807, 486.0 ],
					"source" : [ "obj-366", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-373", 0 ],
					"midpoints" : [ 1609.669562458992004, 333.0, 1609.669562458992004, 333.0 ],
					"source" : [ "obj-367", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-373", 3 ],
					"midpoints" : [ 1668.573667764663696, 300.0, 1650.0, 300.0, 1650.0, 333.0, 1669.669562458992004, 333.0 ],
					"source" : [ "obj-368", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-5", 4 ],
					"source" : [ "obj-369", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-34", 0 ],
					"midpoints" : [ 233.228399991989136, 654.0, 233.228399991989136, 654.0 ],
					"source" : [ "obj-37", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-373", 2 ],
					"midpoints" : [ 1638.436683654785156, 306.0, 1647.0, 306.0, 1647.0, 333.0, 1649.669562458992004, 333.0 ],
					"source" : [ "obj-370", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-373", 1 ],
					"midpoints" : [ 1609.669562458992004, 300.0, 1587.0, 300.0, 1587.0, 333.0, 1629.669562458992004, 333.0 ],
					"source" : [ "obj-371", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-369", 0 ],
					"midpoints" : [ 1689.669562458992004, 375.0, 1665.83394193649292, 375.0 ],
					"source" : [ "obj-373", 2 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-372", 0 ],
					"midpoints" : [ 1649.669562458992004, 375.0, 1650.0, 375.0, 1650.0, 408.0, 1596.0, 408.0, 1596.0, 372.0, 1609.669562458992004, 372.0 ],
					"source" : [ "obj-373", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-425", 0 ],
					"midpoints" : [ 1609.669562458992004, 375.0, 1596.0, 375.0, 1596.0, 411.0, 1257.0, 411.0, 1257.0, 426.0, 1215.0, 426.0, 1215.0, 717.0, 1228.418837547302246, 717.0 ],
					"source" : [ "obj-373", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-373", 4 ],
					"midpoints" : [ 1663.094216108322144, 333.0, 1689.669562458992004, 333.0 ],
					"source" : [ "obj-374", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-381", 0 ],
					"midpoints" : [ 1486.381900191307068, 333.0, 1486.381900191307068, 333.0 ],
					"source" : [ "obj-375", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-381", 3 ],
					"midpoints" : [ 1541.176416754722595, 300.0, 1578.0, 300.0, 1578.0, 333.0, 1546.381900191307068, 333.0 ],
					"source" : [ "obj-376", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-5", 3 ],
					"source" : [ "obj-377", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-381", 2 ],
					"midpoints" : [ 1513.779158473014832, 300.0, 1473.0, 300.0, 1473.0, 333.0, 1526.381900191307068, 333.0 ],
					"source" : [ "obj-378", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-381", 1 ],
					"midpoints" : [ 1486.381900191307068, 300.0, 1473.0, 300.0, 1473.0, 333.0, 1506.381900191307068, 333.0 ],
					"source" : [ "obj-379", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-377", 0 ],
					"midpoints" : [ 1566.381900191307068, 372.0, 1538.436690926551819, 372.0 ],
					"source" : [ "obj-381", 2 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-380", 0 ],
					"midpoints" : [ 1526.381900191307068, 405.0, 1473.0, 405.0, 1473.0, 369.0, 1486.381900191307068, 369.0 ],
					"source" : [ "obj-381", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-425", 0 ],
					"midpoints" : [ 1486.381900191307068, 372.0, 1473.0, 372.0, 1473.0, 411.0, 1257.0, 411.0, 1257.0, 426.0, 1215.0, 426.0, 1215.0, 717.0, 1228.418837547302246, 717.0 ],
					"source" : [ "obj-381", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-381", 4 ],
					"midpoints" : [ 1534.327102184295654, 333.0, 1566.381900191307068, 333.0 ],
					"source" : [ "obj-382", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-389", 0 ],
					"midpoints" : [ 1357.614786267280579, 333.0, 1357.614786267280579, 333.0 ],
					"source" : [ "obj-383", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-389", 3 ],
					"midpoints" : [ 1413.779165744781494, 300.0, 1395.0, 300.0, 1395.0, 333.0, 1417.614786267280579, 333.0 ],
					"source" : [ "obj-384", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-5", 2 ],
					"source" : [ "obj-385", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-389", 2 ],
					"midpoints" : [ 1386.38190746307373, 300.0, 1395.0, 300.0, 1395.0, 330.0, 1397.614786267280579, 330.0 ],
					"source" : [ "obj-386", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-389", 1 ],
					"midpoints" : [ 1357.614786267280579, 300.0, 1344.0, 300.0, 1344.0, 333.0, 1377.614786267280579, 333.0 ],
					"source" : [ "obj-387", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-385", 0 ],
					"midpoints" : [ 1437.614786267280579, 372.0, 1415.149028658866882, 372.0 ],
					"source" : [ "obj-389", 2 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-388", 0 ],
					"midpoints" : [ 1397.614786267280579, 372.0, 1357.614786267280579, 372.0 ],
					"source" : [ "obj-389", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-425", 0 ],
					"midpoints" : [ 1357.614786267280579, 372.0, 1344.0, 372.0, 1344.0, 411.0, 1257.0, 411.0, 1257.0, 426.0, 1215.0, 426.0, 1215.0, 717.0, 1228.418837547302246, 717.0 ],
					"source" : [ "obj-389", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-24", 0 ],
					"midpoints" : [ 192.695649147033691, 1077.0, 188.804344177246094, 1077.0 ],
					"source" : [ "obj-39", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-389", 4 ],
					"midpoints" : [ 1409.66957700252533, 333.0, 1437.614786267280579, 333.0 ],
					"source" : [ "obj-390", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-397", 0 ],
					"midpoints" : [ 1238.436712741851807, 333.0, 1238.436712741851807, 333.0 ],
					"source" : [ "obj-391", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-397", 3 ],
					"midpoints" : [ 1294.601092219352722, 300.0, 1275.0, 300.0, 1275.0, 333.0, 1298.436712741851807, 333.0 ],
					"source" : [ "obj-392", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-5", 1 ],
					"source" : [ "obj-393", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-397", 2 ],
					"midpoints" : [ 1268.573696851730347, 300.0, 1275.0, 300.0, 1275.0, 330.0, 1278.436712741851807, 330.0 ],
					"source" : [ "obj-394", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-397", 1 ],
					"midpoints" : [ 1238.436712741851807, 300.0, 1224.0, 300.0, 1224.0, 330.0, 1258.436712741851807, 330.0 ],
					"source" : [ "obj-395", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-393", 0 ],
					"midpoints" : [ 1318.436712741851807, 372.0, 1329.0, 372.0, 1329.0, 408.0, 1275.0, 408.0, 1275.0, 372.0, 1287.751777648925781, 372.0 ],
					"source" : [ "obj-397", 2 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-396", 0 ],
					"midpoints" : [ 1278.436712741851807, 372.0, 1263.0, 372.0, 1263.0, 381.0, 1236.436712741851807, 381.0 ],
					"source" : [ "obj-397", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-425", 0 ],
					"midpoints" : [ 1238.436712741851807, 381.0, 1191.0, 381.0, 1191.0, 603.0, 1228.418837547302246, 603.0 ],
					"source" : [ "obj-397", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-397", 4 ],
					"midpoints" : [ 1290.491503477096558, 333.0, 1318.436712741851807, 333.0 ],
					"source" : [ "obj-398", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-209", 0 ],
					"midpoints" : [ 70.984846115112305, 297.0, 88.156562447547913, 297.0 ],
					"order" : 0,
					"source" : [ "obj-4", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-216", 0 ],
					"midpoints" : [ 70.984846115112305, 297.0, 36.546263337135315, 297.0 ],
					"order" : 1,
					"source" : [ "obj-4", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-111", 1 ],
					"midpoints" : [ 249.0, 105.0, 231.0, 105.0, 231.0, 159.0, 269.5, 159.0 ],
					"source" : [ "obj-40", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-228", 0 ],
					"source" : [ "obj-401", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-409", 0 ],
					"midpoints" : [ 1060.85128116607666, 669.0, 1038.0, 669.0, 1038.0, 726.0, 1060.85128116607666, 726.0 ],
					"source" : [ "obj-402", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-415", 0 ],
					"midpoints" : [ 1349.465232968330383, 702.0, 1349.465232968330383, 702.0 ],
					"source" : [ "obj-403", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-413", 1 ],
					"midpoints" : [ 1266.256672859191895, 786.0, 1215.0, 786.0, 1215.0, 759.0, 1243.418837547302246, 759.0 ],
					"source" : [ "obj-404", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-409", 0 ],
					"midpoints" : [ 1060.85128116607666, 717.0, 1060.85128116607666, 717.0 ],
					"source" : [ "obj-406", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-409", 1 ],
					"midpoints" : [ 1109.499926567077637, 717.0, 1076.35128116607666, 717.0 ],
					"source" : [ "obj-407", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-136", 0 ],
					"source" : [ "obj-409", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-425", 0 ],
					"midpoints" : [ 1060.85128116607666, 753.0, 1203.0, 753.0, 1203.0, 729.0, 1228.418837547302246, 729.0 ],
					"source" : [ "obj-409", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-113", 1 ],
					"midpoints" : [ 329.0, 105.0, 309.0, 105.0, 309.0, 159.0, 348.5, 159.0 ],
					"source" : [ "obj-41", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-409", 2 ],
					"midpoints" : [ 1123.50003319978714, 753.0, 1101.0, 753.0, 1101.0, 726.0, 1091.85128116607666, 726.0 ],
					"source" : [ "obj-410", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-406", 0 ],
					"midpoints" : [ 1091.932360172271729, 669.0, 1060.85128116607666, 669.0 ],
					"order" : 1,
					"source" : [ "obj-411", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-407", 0 ],
					"midpoints" : [ 1091.932360172271729, 669.0, 1109.499926567077637, 669.0 ],
					"order" : 0,
					"source" : [ "obj-411", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-414", 0 ],
					"midpoints" : [ 1228.418837547302246, 786.0, 1228.418837547302246, 786.0 ],
					"source" : [ "obj-413", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-412", 0 ],
					"midpoints" : [ 1228.418837547302246, 816.0, 1228.418837547302246, 816.0 ],
					"order" : 2,
					"source" : [ "obj-414", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-423", 0 ],
					"midpoints" : [ 1228.418837547302246, 816.0, 1290.0, 816.0, 1290.0, 774.0, 1452.548553824424744, 774.0 ],
					"order" : 0,
					"source" : [ "obj-414", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-424", 0 ],
					"midpoints" : [ 1228.418837547302246, 822.0, 1215.0, 822.0, 1215.0, 846.0, 1229.770188808441162, 846.0 ],
					"order" : 1,
					"source" : [ "obj-414", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-419", 0 ],
					"midpoints" : [ 1349.465232968330383, 729.0, 1383.0, 729.0, 1383.0, 702.0, 1397.381897807121277, 702.0 ],
					"source" : [ "obj-415", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-421", 0 ],
					"midpoints" : [ 1369.465232968330383, 774.0, 1530.0, 774.0, 1530.0, 750.0, 1653.0, 750.0, 1653.0, 720.0, 1676.548553824424744, 720.0 ],
					"source" : [ "obj-415", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-417", 0 ],
					"midpoints" : [ 1397.381897807121277, 669.0, 1397.381897807121277, 669.0 ],
					"source" : [ "obj-416", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-419", 1 ],
					"midpoints" : [ 1411.381897807121277, 699.0, 1433.381897807121277, 699.0 ],
					"source" : [ "obj-417", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-420", 0 ],
					"midpoints" : [ 1397.381897807121277, 774.0, 1434.0, 774.0, 1434.0, 744.0, 1457.798562169075012, 744.0 ],
					"source" : [ "obj-418", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-418", 0 ],
					"midpoints" : [ 1397.381897807121277, 732.0, 1397.381897807121277, 732.0 ],
					"source" : [ "obj-419", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-423", 0 ],
					"midpoints" : [ 1457.798562169075012, 771.0, 1452.548553824424744, 771.0 ],
					"source" : [ "obj-420", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-422", 0 ],
					"midpoints" : [ 1676.548553824424744, 750.0, 1676.548553824424744, 750.0 ],
					"source" : [ "obj-421", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-423", 1 ],
					"midpoints" : [ 1676.548553824424744, 771.0, 1700.548553824424744, 771.0 ],
					"source" : [ "obj-422", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-413", 0 ],
					"midpoints" : [ 1228.418837547302246, 756.0, 1228.418837547302246, 756.0 ],
					"source" : [ "obj-425", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-423", 0 ],
					"midpoints" : [ 1457.798562169075012, 741.0, 1443.0, 741.0, 1443.0, 771.0, 1452.548553824424744, 771.0 ],
					"source" : [ "obj-426", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-423", 0 ],
					"midpoints" : [ 1457.798562169075012, 711.0, 1443.0, 711.0, 1443.0, 771.0, 1452.548553824424744, 771.0 ],
					"source" : [ "obj-427", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-423", 0 ],
					"midpoints" : [ 1457.798562169075012, 684.0, 1635.0, 684.0, 1635.0, 765.0, 1518.0, 765.0, 1518.0, 774.0, 1452.548553824424744, 774.0 ],
					"source" : [ "obj-428", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-2", 0 ],
					"midpoints" : [ 371.525311708450317, 372.0, 291.0, 372.0, 291.0, 495.0, 43.346157073974609, 495.0 ],
					"source" : [ "obj-43", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-231", 0 ],
					"source" : [ "obj-43", 2 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-46", 0 ],
					"midpoints" : [ 411.525311708450317, 408.0, 348.0, 408.0, 348.0, 369.0, 371.525311708450317, 369.0 ],
					"source" : [ "obj-43", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-433", 0 ],
					"midpoints" : [ 1077.340417861938477, 113.558743357658386, 1081.840417861938477, 113.558743357658386 ],
					"source" : [ "obj-432", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-228", 0 ],
					"source" : [ "obj-5", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-115", 1 ],
					"midpoints" : [ 408.0, 105.0, 390.0, 105.0, 390.0, 159.0, 428.5, 159.0 ],
					"source" : [ "obj-50", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-117", 1 ],
					"midpoints" : [ 486.0, 105.0, 468.0, 105.0, 468.0, 159.0, 506.5, 159.0 ],
					"source" : [ "obj-51", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-121", 1 ],
					"midpoints" : [ 564.0, 105.0, 546.0, 105.0, 546.0, 159.0, 586.5, 159.0 ],
					"source" : [ "obj-52", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-123", 1 ],
					"midpoints" : [ 644.0, 105.0, 624.0, 105.0, 624.0, 159.0, 664.5, 159.0 ],
					"source" : [ "obj-56", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-43", 4 ],
					"midpoints" : [ 423.424045205116272, 333.0, 451.525311708450317, 333.0 ],
					"source" : [ "obj-65", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-125", 1 ],
					"midpoints" : [ 725.0, 105.0, 705.0, 105.0, 705.0, 159.0, 743.5, 159.0 ],
					"source" : [ "obj-67", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-43", 1 ],
					"midpoints" : [ 371.525311708450317, 300.0, 357.0, 300.0, 357.0, 330.0, 391.525311708450317, 330.0 ],
					"source" : [ "obj-69", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-228", 0 ],
					"midpoints" : [ 39.846160888671875, 474.0, 241.145566582679749, 474.0 ],
					"source" : [ "obj-7", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-22", 1 ],
					"midpoints" : [ 83.346160888671875, 651.0, 69.0, 651.0, 69.0, 624.0, 58.346157073974609, 624.0 ],
					"source" : [ "obj-70", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-43", 2 ],
					"midpoints" : [ 400.639235377311707, 303.0, 411.525311708450317, 303.0 ],
					"source" : [ "obj-71", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-43", 3 ],
					"midpoints" : [ 427.221513509750366, 300.0, 408.0, 300.0, 408.0, 333.0, 431.525311708450317, 333.0 ],
					"source" : [ "obj-74", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-32", 0 ],
					"midpoints" : [ 216.608692169189453, 1227.0, 147.0, 1227.0, 147.0, 1260.0, 49.217391014099121, 1260.0 ],
					"source" : [ "obj-75", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-66", 0 ],
					"midpoints" : [ 103.565216064453125, 1245.0, 21.0, 1245.0, 21.0, 1446.0, 49.217391014099121, 1446.0 ],
					"source" : [ "obj-76", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-131", 0 ],
					"midpoints" : [ 49.217391014099121, 1041.0, 49.217391014099121, 1041.0 ],
					"order" : 1,
					"source" : [ "obj-77", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-211", 0 ],
					"midpoints" : [ 49.217391014099121, 1041.0, 168.0, 1041.0, 168.0, 1017.0, 192.695649147033691, 1017.0 ],
					"order" : 0,
					"source" : [ "obj-77", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-75", 0 ],
					"midpoints" : [ 216.608692169189453, 1200.0, 216.608692169189453, 1200.0 ],
					"source" : [ "obj-78", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-109", 1 ],
					"midpoints" : [ 172.0, 105.0, 153.0, 105.0, 153.0, 159.0, 192.5, 159.0 ],
					"source" : [ "obj-8", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-221", 0 ],
					"midpoints" : [ 37.0, 105.0, 42.5, 105.0 ],
					"source" : [ "obj-82", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-13", 0 ],
					"midpoints" : [ 96.5, 186.0, 72.0, 186.0, 72.0, 36.0, 112.5, 36.0 ],
					"order" : 0,
					"source" : [ "obj-84", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-244", 0 ],
					"order" : 1,
					"source" : [ "obj-84", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-84", 0 ],
					"midpoints" : [ 96.5, 162.0, 96.5, 162.0 ],
					"order" : 0,
					"source" : [ "obj-85", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-89", 0 ],
					"midpoints" : [ 96.5, 159.0, 72.0, 159.0, 72.0, 18.0, 91.0, 18.0 ],
					"order" : 1,
					"source" : [ "obj-85", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-133", 0 ],
					"midpoints" : [ 97.043477058410645, 1041.0, 83.999999046325684, 1041.0 ],
					"order" : 1,
					"source" : [ "obj-87", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-217", 0 ],
					"midpoints" : [ 97.043477058410645, 1041.0, 168.0, 1041.0, 168.0, 1005.0, 225.304344177246094, 1005.0 ],
					"order" : 0,
					"source" : [ "obj-87", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-157", 0 ],
					"midpoints" : [ 186.314815998077393, 687.0, 186.314815998077393, 687.0 ],
					"source" : [ "obj-88", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-3", 0 ],
					"midpoints" : [ 91.0, 33.0, 92.0, 33.0 ],
					"source" : [ "obj-89", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-6", 0 ],
					"midpoints" : [ 296.191367983818054, 726.0, 273.0, 726.0, 273.0, 762.0, 290.709903955459595, 762.0 ],
					"source" : [ "obj-9", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-125", 2 ],
					"midpoints" : [ 729.5, 135.0, 714.0, 135.0, 714.0, 159.0, 759.5, 159.0 ],
					"source" : [ "obj-99", 0 ]
				}

			}
 ],
		"parameters" : 		{
			"obj-135" : [ "freqFine/2", "dial", 0 ],
			"obj-137" : [ "incHarm/2", "button[1]", 0 ],
			"obj-138" : [ "decHarm/2", "button", 0 ],
			"obj-143" : [ "freqFine/3", "dial", 0 ],
			"obj-145" : [ "incHarm/3", "button[1]", 0 ],
			"obj-146" : [ "decHarm/3", "button", 0 ],
			"obj-151" : [ "freqFine/4", "dial", 0 ],
			"obj-153" : [ "incHarm/4", "button[1]", 0 ],
			"obj-154" : [ "decHarm/4", "button", 0 ],
			"obj-160" : [ "freqFine/5", "dial", 0 ],
			"obj-162" : [ "incHarm/5", "button[1]", 0 ],
			"obj-163" : [ "decHarm/5", "button", 0 ],
			"obj-176" : [ "freqFine/0", "dial", 0 ],
			"obj-178" : [ "incHarm/0", "button[1]", 0 ],
			"obj-179" : [ "decHarm/0", "button", 0 ],
			"obj-184" : [ "freqFine/8", "dial", 0 ],
			"obj-186" : [ "incHarm/8", "button[1]", 0 ],
			"obj-187" : [ "decHarm/8", "button", 0 ],
			"obj-193" : [ "freqFine/7", "dial", 0 ],
			"obj-195" : [ "incHarm/7", "button[1]", 0 ],
			"obj-196" : [ "decHarm/7", "button", 0 ],
			"obj-201" : [ "freqFine/6", "dial", 0 ],
			"obj-203" : [ "incHarm/6", "button[1]", 0 ],
			"obj-204" : [ "decHarm/6", "button", 0 ],
			"obj-209" : [ "fundFine", "dial", 0 ],
			"obj-216" : [ "fundCoarse", "dial", 0 ],
			"obj-230" : [ "live.numbox", "live.numbox", 0 ],
			"obj-231" : [ "live.numbox[1]", "live.numbox", 0 ],
			"obj-233" : [ "live.numbox[2]", "live.numbox", 0 ],
			"obj-234" : [ "live.numbox[3]", "live.numbox", 0 ],
			"obj-235" : [ "live.numbox[4]", "live.numbox", 0 ],
			"obj-236" : [ "live.numbox[5]", "live.numbox", 0 ],
			"obj-282" : [ "harmAmpA/8", "slider[9]", 0 ],
			"obj-283" : [ "harmAmpA/7", "slider[8]", 0 ],
			"obj-284" : [ "harmAmpA/6", "slider[7]", 0 ],
			"obj-285" : [ "harmAmpA/5", "slider[6]", 0 ],
			"obj-286" : [ "harmAmpA/4", "slider[5]", 0 ],
			"obj-287" : [ "harmAmpA/3", "slider[4]", 0 ],
			"obj-288" : [ "harmAmpA/2", "slider[3]", 0 ],
			"obj-289" : [ "harmAmpA/1", "slider[2]", 0 ],
			"obj-290" : [ "harmAmpA/0", "slider[1]", 0 ],
			"obj-3" : [ "harmAmp/0", "slider[1]", 0 ],
			"obj-328" : [ "freqFineA/0", "dial", 0 ],
			"obj-330" : [ "incHarmA/0", "button[1]", 0 ],
			"obj-331" : [ "decHarmA/0", "button", 0 ],
			"obj-336" : [ "freqFineA/8", "dial", 0 ],
			"obj-338" : [ "incHarmA/8", "button[1]", 0 ],
			"obj-339" : [ "decHarmA/8", "button", 0 ],
			"obj-344" : [ "freqFineA/7", "dial", 0 ],
			"obj-346" : [ "incHarmA/7", "button[1]", 0 ],
			"obj-347" : [ "decHarmA/7", "button", 0 ],
			"obj-352" : [ "freqFineA/6", "dial", 0 ],
			"obj-354" : [ "incHarmA/6", "button[1]", 0 ],
			"obj-355" : [ "decHarmA/6", "button", 0 ],
			"obj-360" : [ "freqFineA/5", "dial", 0 ],
			"obj-362" : [ "incHarmA/5", "button[1]", 0 ],
			"obj-363" : [ "decHarmA/5", "button", 0 ],
			"obj-368" : [ "freqFineA/4", "dial", 0 ],
			"obj-370" : [ "incHarmA/4", "button[1]", 0 ],
			"obj-371" : [ "decHarmA/4", "button", 0 ],
			"obj-376" : [ "freqFineA/3", "dial", 0 ],
			"obj-378" : [ "incHarmA/3", "button[1]", 0 ],
			"obj-379" : [ "decHarmA/3", "button", 0 ],
			"obj-384" : [ "freqFineA/2", "dial", 0 ],
			"obj-386" : [ "incHarmA/2", "button[1]", 0 ],
			"obj-387" : [ "decHarmA/2", "button", 0 ],
			"obj-392" : [ "freqFineA/1", "dial", 0 ],
			"obj-394" : [ "incHarmA/1", "button[1]", 0 ],
			"obj-395" : [ "decHarmA/1", "button", 0 ],
			"obj-40" : [ "harmAmp/2", "slider[3]", 0 ],
			"obj-404" : [ "mainAmpA", "slider", 0 ],
			"obj-406" : [ "fundCoarseA", "dial", 0 ],
			"obj-407" : [ "fundFineA", "dial", 0 ],
			"obj-41" : [ "harmAmp/3", "slider[4]", 0 ],
			"obj-432" : [ "ampFundA", "slider[1]", 0 ],
			"obj-50" : [ "harmAmp/4", "slider[5]", 0 ],
			"obj-51" : [ "harmAmp/5", "slider[6]", 0 ],
			"obj-52" : [ "harmAmp/6", "slider[7]", 0 ],
			"obj-56" : [ "harmAmp/7", "slider[8]", 0 ],
			"obj-67" : [ "harmAmp/8", "slider[9]", 0 ],
			"obj-69" : [ "decHarm/1", "button", 0 ],
			"obj-70" : [ "mainAmp", "slider", 0 ],
			"obj-71" : [ "incHarm/1", "button[1]", 0 ],
			"obj-74" : [ "freqFine/1", "dial", 0 ],
			"obj-8" : [ "harmAmp/1", "slider[2]", 0 ],
			"obj-82" : [ "ampFund", "slider[1]", 0 ],
			"parameterbanks" : 			{
				"0" : 				{
					"index" : 0,
					"name" : "",
					"parameters" : [ "-", "-", "-", "-", "-", "-", "-", "-" ]
				}

			}
,
			"inherited_shortname" : 1
		}
,
		"dependency_cache" : [ 			{
				"name" : "fundSet.maxpat",
				"bootpath" : "~/Library/CloudStorage/GoogleDrive-austin.tecks@gmail.com/My Drive/MDM/schizophonic_remediation/MaxPatches",
				"patcherrelativepath" : ".",
				"type" : "JSON",
				"implicit" : 1
			}
, 			{
				"name" : "gridList.maxpat",
				"bootpath" : "~/Library/CloudStorage/GoogleDrive-austin.tecks@gmail.com/My Drive/MDM/schizophonic_remediation/MaxPatches",
				"patcherrelativepath" : ".",
				"type" : "JSON",
				"implicit" : 1
			}
, 			{
				"name" : "harmModTouch.maxpat",
				"bootpath" : "~/Library/CloudStorage/GoogleDrive-austin.tecks@gmail.com/My Drive/MDM/schizophonic_remediation/MaxPatches",
				"patcherrelativepath" : ".",
				"type" : "JSON",
				"implicit" : 1
			}
, 			{
				"name" : "o.route.mxo",
				"type" : "iLaX"
			}
 ],
		"autosave" : 0,
		"styles" : [ 			{
				"name" : "rnbohighcontrast",
				"default" : 				{
					"accentcolor" : [ 0.666666666666667, 0.666666666666667, 0.666666666666667, 1.0 ],
					"bgcolor" : [ 0.0, 0.0, 0.0, 1.0 ],
					"bgfillcolor" : 					{
						"angle" : 270.0,
						"autogradient" : 0.0,
						"color" : [ 0.0, 0.0, 0.0, 1.0 ],
						"color1" : [ 0.090196078431373, 0.090196078431373, 0.090196078431373, 1.0 ],
						"color2" : [ 0.156862745098039, 0.168627450980392, 0.164705882352941, 1.0 ],
						"proportion" : 0.5,
						"type" : "color"
					}
,
					"clearcolor" : [ 1.0, 1.0, 1.0, 0.0 ],
					"color" : [ 1.0, 0.874509803921569, 0.141176470588235, 1.0 ],
					"editing_bgcolor" : [ 0.258823529411765, 0.258823529411765, 0.258823529411765, 1.0 ],
					"elementcolor" : [ 0.223386004567146, 0.254748553037643, 0.998085916042328, 1.0 ],
					"fontsize" : [ 13.0 ],
					"locked_bgcolor" : [ 0.258823529411765, 0.258823529411765, 0.258823529411765, 1.0 ],
					"selectioncolor" : [ 0.301960784313725, 0.694117647058824, 0.949019607843137, 1.0 ],
					"stripecolor" : [ 0.258823529411765, 0.258823529411765, 0.258823529411765, 1.0 ],
					"textcolor" : [ 1.0, 1.0, 1.0, 1.0 ],
					"textcolor_inverse" : [ 1.0, 1.0, 1.0, 1.0 ]
				}
,
				"parentstyle" : "",
				"multi" : 0
			}
, 			{
				"name" : "rnbolight",
				"default" : 				{
					"accentcolor" : [ 0.443137254901961, 0.505882352941176, 0.556862745098039, 1.0 ],
					"bgcolor" : [ 0.796078431372549, 0.862745098039216, 0.925490196078431, 1.0 ],
					"bgfillcolor" : 					{
						"angle" : 270.0,
						"autogradient" : 0.0,
						"color" : [ 0.835294117647059, 0.901960784313726, 0.964705882352941, 1.0 ],
						"color1" : [ 0.031372549019608, 0.125490196078431, 0.211764705882353, 1.0 ],
						"color2" : [ 0.263682, 0.004541, 0.038797, 1.0 ],
						"proportion" : 0.39,
						"type" : "color"
					}
,
					"clearcolor" : [ 0.898039, 0.898039, 0.898039, 1.0 ],
					"color" : [ 0.815686274509804, 0.509803921568627, 0.262745098039216, 1.0 ],
					"editing_bgcolor" : [ 0.898039, 0.898039, 0.898039, 1.0 ],
					"elementcolor" : [ 0.337254901960784, 0.384313725490196, 0.462745098039216, 1.0 ],
					"fontname" : [ "Lato" ],
					"locked_bgcolor" : [ 0.898039, 0.898039, 0.898039, 1.0 ],
					"stripecolor" : [ 0.309803921568627, 0.698039215686274, 0.764705882352941, 1.0 ],
					"textcolor_inverse" : [ 0.0, 0.0, 0.0, 1.0 ]
				}
,
				"parentstyle" : "",
				"multi" : 0
			}
, 			{
				"name" : "rnbomonokai",
				"default" : 				{
					"accentcolor" : [ 0.501960784313725, 0.501960784313725, 0.501960784313725, 1.0 ],
					"bgcolor" : [ 0.0, 0.0, 0.0, 1.0 ],
					"bgfillcolor" : 					{
						"angle" : 270.0,
						"autogradient" : 0.0,
						"color" : [ 0.0, 0.0, 0.0, 1.0 ],
						"color1" : [ 0.031372549019608, 0.125490196078431, 0.211764705882353, 1.0 ],
						"color2" : [ 0.263682, 0.004541, 0.038797, 1.0 ],
						"proportion" : 0.39,
						"type" : "color"
					}
,
					"clearcolor" : [ 0.976470588235294, 0.96078431372549, 0.917647058823529, 1.0 ],
					"color" : [ 0.611764705882353, 0.125490196078431, 0.776470588235294, 1.0 ],
					"editing_bgcolor" : [ 0.976470588235294, 0.96078431372549, 0.917647058823529, 1.0 ],
					"elementcolor" : [ 0.749019607843137, 0.83921568627451, 1.0, 1.0 ],
					"fontname" : [ "Lato" ],
					"locked_bgcolor" : [ 0.976470588235294, 0.96078431372549, 0.917647058823529, 1.0 ],
					"stripecolor" : [ 0.796078431372549, 0.207843137254902, 1.0, 1.0 ],
					"textcolor" : [ 0.129412, 0.129412, 0.129412, 1.0 ]
				}
,
				"parentstyle" : "",
				"multi" : 0
			}
 ],
		"elementcolor" : [ 0.831372549019608, 0.831372549019608, 0.831372549019608, 1.0 ],
		"bgcolor" : [ 0.333333333333333, 0.333333333333333, 0.333333333333333, 1.0 ],
		"oscsendudpaddr" : "localhost"
	}

}
