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
		"rect" : [ 609.0, 86.0, 273.0, 717.0 ],
		"openinpresentation" : 1,
		"gridsize" : [ 15.0, 15.0 ],
		"toolbars_unpinned_last_save" : 8,
		"boxes" : [ 			{
				"box" : 				{
					"id" : "obj-15",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 312.938080370426178, 117.647056579589844, 24.0, 24.0 ],
					"presentation" : 1,
					"presentation_rect" : [ -278.102188289165497, 506.0, 24.0, 24.0 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_enum" : [ "off", "on" ],
							"parameter_longname" : "button[7]",
							"parameter_mmax" : 1,
							"parameter_modmode" : 0,
							"parameter_shortname" : "button[7]",
							"parameter_type" : 2
						}

					}
,
					"varname" : "button[7]"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-51",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 474.999981880187988, 612.499976634979248, 50.0, 22.0 ],
					"text" : "1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-49",
					"items" : [ "Unscaled", ",", "Scaled" ],
					"maxclass" : "umenu",
					"numinlets" : 1,
					"numoutlets" : 3,
					"outlettype" : [ "int", "", "" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 412.079999999999984, 563.899999999999977, 100.0, 22.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 693.388391315937042, 757.024751424789429, 100.0, 22.0 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_enum" : [ "Unscaled", "Scaled" ],
							"parameter_longname" : "umenu",
							"parameter_mmax" : 1,
							"parameter_modmode" : 0,
							"parameter_shortname" : "umenu",
							"parameter_type" : 2
						}

					}
,
					"varname" : "umenu"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-21",
					"maxclass" : "gridmeter~",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "multichannelsignal" ],
					"patching_rect" : [ 565.960000000000036, 399.160000000000025, 20.0, 16.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 510.743773341178894, 935.537138223648071, 20.0, 16.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-431",
					"lastchannelcount" : 0,
					"maxclass" : "live.gain~",
					"numinlets" : 2,
					"numoutlets" : 5,
					"outlettype" : [ "signal", "signal", "", "float", "list" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 2418.548404395580292, 1074.193556070327759, 48.0, 136.0 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_longname" : "live.gain~",
							"parameter_mmax" : 6.0,
							"parameter_mmin" : -70.0,
							"parameter_modmode" : 3,
							"parameter_shortname" : "live.gain~",
							"parameter_type" : 0,
							"parameter_unitstyle" : 4
						}

					}
,
					"varname" : "live.gain~"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-429",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "multichannelsignal" ],
					"patching_rect" : [ 2404.0, 937.755093097686768, 201.0, 22.0 ],
					"text" : "mc.mixdown~ @chans 1 @autogain"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-350",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 3016.0, 72.0, 29.5, 22.0 ],
					"text" : "0."
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-351",
					"maxclass" : "newobj",
					"numinlets" : 3,
					"numoutlets" : 1,
					"outlettype" : [ "signal" ],
					"patching_rect" : [ 2388.0, 692.0, 60.0, 22.0 ],
					"text" : "clip~ 0. 1."
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-353",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 2388.0, 728.0, 45.0, 22.0 ],
					"text" : "dac~ 5"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-354",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"patching_rect" : [ 3124.0, 228.0, 29.5, 22.0 ],
					"text" : "+ 1."
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-355",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"patching_rect" : [ 3240.0, 228.0, 29.5, 22.0 ],
					"text" : "+ 1."
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-356",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"patching_rect" : [ 3068.0, 228.0, 29.5, 22.0 ],
					"text" : "+ 1."
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-357",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"patching_rect" : [ 3180.0, 232.0, 29.5, 22.0 ],
					"text" : "+ 1."
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-358",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"patching_rect" : [ 3240.0, 120.0, 29.5, 22.0 ],
					"text" : "+ 1."
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-359",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"patching_rect" : [ 3184.0, 120.0, 29.5, 22.0 ],
					"text" : "+ 1."
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-360",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"patching_rect" : [ 3124.0, 120.0, 29.5, 22.0 ],
					"text" : "+ 1."
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-361",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"patching_rect" : [ 3068.0, 116.0, 29.5, 22.0 ],
					"text" : "+ 1."
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-362",
					"maxclass" : "newobj",
					"numinlets" : 8,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 3304.0, 128.0, 92.5, 22.0 ],
					"text" : "pak i i i i i i i i"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 8.0,
					"id" : "obj-363",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 3180.0, 196.0, 27.0, 35.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 290.531551071575734, 113.953484296798706, 27.0, 35.0 ],
					"saved_attribute_attributes" : 					{
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "-[40]",
							"parameter_mmax" : 0.02,
							"parameter_mmin" : -0.02,
							"parameter_modmode" : 4,
							"parameter_shortname" : "1",
							"parameter_type" : 0,
							"parameter_unitstyle" : 1
						}

					}
,
					"showname" : 0,
					"textcolor" : [ 0.086275, 0.309804, 0.52549, 1.0 ],
					"varname" : "live.dial[34]"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 8.0,
					"id" : "obj-364",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 3124.0, 196.0, 27.0, 35.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 255.48171843801228, 113.953484296798706, 27.0, 35.0 ],
					"saved_attribute_attributes" : 					{
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "-[41]",
							"parameter_mmax" : 0.02,
							"parameter_mmin" : -0.02,
							"parameter_modmode" : 4,
							"parameter_shortname" : "1",
							"parameter_type" : 0,
							"parameter_unitstyle" : 1
						}

					}
,
					"showname" : 0,
					"textcolor" : [ 0.086275, 0.309804, 0.52549, 1.0 ],
					"varname" : "live.dial[35]"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 8.0,
					"id" : "obj-365",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 3068.0, 196.0, 27.0, 35.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 220.431885804448825, 113.953484296798706, 27.0, 35.0 ],
					"saved_attribute_attributes" : 					{
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "-[28]",
							"parameter_mmax" : 0.02,
							"parameter_mmin" : -0.02,
							"parameter_modmode" : 4,
							"parameter_shortname" : "1",
							"parameter_type" : 0,
							"parameter_unitstyle" : 1
						}

					}
,
					"showname" : 0,
					"textcolor" : [ 0.086275, 0.309804, 0.52549, 1.0 ],
					"varname" : "live.dial[36]"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 8.0,
					"id" : "obj-366",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 3240.0, 84.0, 27.0, 35.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 185.382053170885371, 113.953484296798706, 27.0, 35.0 ],
					"saved_attribute_attributes" : 					{
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "-[42]",
							"parameter_mmax" : 0.02,
							"parameter_mmin" : -0.02,
							"parameter_modmode" : 4,
							"parameter_shortname" : "1",
							"parameter_type" : 0,
							"parameter_unitstyle" : 1
						}

					}
,
					"showname" : 0,
					"textcolor" : [ 0.086275, 0.309804, 0.52549, 1.0 ],
					"varname" : "live.dial[37]"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-367",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 3180.0, 256.0, 32.0, 22.0 ],
					"text" : "7 $1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-368",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 3124.0, 256.0, 32.0, 22.0 ],
					"text" : "6 $1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-369",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 3068.0, 256.0, 32.0, 22.0 ],
					"text" : "5 $1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-370",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 3240.0, 152.0, 32.0, 22.0 ],
					"text" : "4 $1"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 8.0,
					"id" : "obj-371",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 3240.0, 196.0, 27.0, 35.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 325.58138370513916, 113.953484296798706, 27.0, 35.0 ],
					"saved_attribute_attributes" : 					{
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "-[43]",
							"parameter_mmax" : 0.02,
							"parameter_mmin" : -0.02,
							"parameter_modmode" : 4,
							"parameter_shortname" : "1",
							"parameter_type" : 0,
							"parameter_unitstyle" : 1
						}

					}
,
					"showname" : 0,
					"textcolor" : [ 0.086275, 0.309804, 0.52549, 1.0 ],
					"varname" : "live.dial[38]"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 8.0,
					"id" : "obj-372",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 3184.0, 84.0, 27.0, 35.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 150.332220537321916, 113.953484296798706, 27.0, 35.0 ],
					"saved_attribute_attributes" : 					{
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "-[44]",
							"parameter_mmax" : 0.02,
							"parameter_mmin" : -0.02,
							"parameter_modmode" : 4,
							"parameter_shortname" : "1",
							"parameter_type" : 0,
							"parameter_unitstyle" : 1
						}

					}
,
					"showname" : 0,
					"textcolor" : [ 0.086275, 0.309804, 0.52549, 1.0 ],
					"varname" : "live.dial[39]"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 8.0,
					"id" : "obj-373",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 3124.0, 84.0, 27.0, 35.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 115.282387903758462, 113.953484296798706, 27.0, 35.0 ],
					"saved_attribute_attributes" : 					{
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "-[45]",
							"parameter_mmax" : 0.02,
							"parameter_mmin" : -0.02,
							"parameter_modmode" : 4,
							"parameter_shortname" : "1",
							"parameter_type" : 0,
							"parameter_unitstyle" : 1
						}

					}
,
					"showname" : 0,
					"textcolor" : [ 0.086275, 0.309804, 0.52549, 1.0 ],
					"varname" : "live.dial[40]"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 8.0,
					"id" : "obj-374",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 3068.0, 84.0, 27.0, 35.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 80.232555270195007, 113.953484296798706, 27.0, 35.0 ],
					"saved_attribute_attributes" : 					{
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "-[46]",
							"parameter_mmax" : 0.02,
							"parameter_mmin" : -0.02,
							"parameter_modmode" : 4,
							"parameter_shortname" : "1",
							"parameter_type" : 0,
							"parameter_unitstyle" : 1
						}

					}
,
					"showname" : 0,
					"textcolor" : [ 0.086275, 0.309804, 0.52549, 1.0 ],
					"varname" : "live.dial[41]"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-375",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 3240.0, 256.0, 32.0, 22.0 ],
					"text" : "8 $1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-376",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 3180.0, 152.0, 32.0, 22.0 ],
					"text" : "3 $1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-377",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 3124.0, 152.0, 32.0, 22.0 ],
					"text" : "2 $1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-378",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 3064.0, 152.0, 32.0, 22.0 ],
					"text" : "1 $1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-379",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 3,
					"outlettype" : [ "multichannelsignal", "", "" ],
					"patching_rect" : [ 3064.0, 340.0, 111.0, 22.0 ],
					"text" : "mc.line~ @chans 8"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-380",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 3068.0, 312.0, 107.0, 22.0 ],
					"text" : "setvalue $1 $2 25."
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-381",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "int" ],
					"patching_rect" : [ 2300.0, 240.0, 33.0, 22.0 ],
					"text" : "== 0"
				}

			}
, 			{
				"box" : 				{
					"attr" : "ignoreclick",
					"fontsize" : 8.0,
					"id" : "obj-382",
					"maxclass" : "attrui",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 2300.0, 264.0, 130.058997750282288, 17.0 ]
				}

			}
, 			{
				"box" : 				{
					"activebgcolor" : [ 0.687607561603021, 0.672947940130859, 0.658307756166483, 1.0 ],
					"id" : "obj-383",
					"maxclass" : "live.numbox",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 2476.0, 272.0, 44.0, 15.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 185.0, 465.0, 44.0, 15.0 ],
					"saved_attribute_attributes" : 					{
						"activebgcolor" : 						{
							"expression" : "themecolor.live_surface_bg"
						}
,
						"valueof" : 						{
							"parameter_longname" : "live.numbox[2]",
							"parameter_mmax" : 1.0,
							"parameter_modmode" : 3,
							"parameter_shortname" : "live.numbox",
							"parameter_type" : 0,
							"parameter_unitstyle" : 1
						}

					}
,
					"varname" : "live.numbox[2]"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-384",
					"maxclass" : "newobj",
					"numinlets" : 3,
					"numoutlets" : 2,
					"outlettype" : [ "bang", "" ],
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
						"rect" : [ 779.0, 62.0, 657.0, 860.0 ],
						"gridsize" : [ 15.0, 15.0 ],
						"boxes" : [ 							{
								"box" : 								{
									"id" : "obj-71",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "bang", "float" ],
									"patching_rect" : [ 362.666677474975586, 89.333335995674133, 29.5, 22.0 ],
									"text" : "t b f"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-67",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "bang", "int" ],
									"patching_rect" : [ 70.000002086162567, 138.666670799255371, 29.5, 22.0 ],
									"text" : "t b i"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-66",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "int" ],
									"patching_rect" : [ 110.666669964790344, 278.666674971580505, 29.5, 22.0 ],
									"text" : "+ 1"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-61",
									"maxclass" : "newobj",
									"numinlets" : 3,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 110.666669964790344, 304.666675746440887, 111.333336651325226, 22.0 ],
									"text" : "switch 2 1"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-59",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "float" ],
									"patching_rect" : [ 248.000007390975952, 166.666671633720398, 33.0, 22.0 ],
									"text" : "* 0.8"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-58",
									"maxclass" : "newobj",
									"numinlets" : 3,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 248.000007390975952, 200.000005960464478, 53.0, 22.0 ],
									"text" : "clip 1. 6."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-54",
									"maxclass" : "newobj",
									"numinlets" : 6,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 202.666672706604004, 258.666674375534058, 133.0, 22.0 ],
									"text" : "scale 0. 1. 0. 1."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-51",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 2,
									"outlettype" : [ "", "" ],
									"patching_rect" : [ 247.666672706604004, 114.666670083999634, 43.0, 22.0 ],
									"text" : "zl.sum"
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-73",
									"index" : 1,
									"maxclass" : "inlet",
									"numinlets" : 0,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 70.000002086162567, 49.333334803581238, 30.0, 30.0 ]
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-74",
									"index" : 2,
									"maxclass" : "inlet",
									"numinlets" : 0,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 186.666672229766846, 49.333334803581238, 30.0, 30.0 ]
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-75",
									"index" : 3,
									"maxclass" : "inlet",
									"numinlets" : 0,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 362.666677474975586, 49.333334803581238, 30.0, 30.0 ]
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-76",
									"index" : 1,
									"maxclass" : "outlet",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 69.750002086162567, 359.333344042301178, 30.0, 30.0 ]
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-77",
									"index" : 2,
									"maxclass" : "outlet",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 110.666669964790344, 359.333344042301178, 30.0, 30.0 ]
								}

							}
 ],
						"lines" : [ 							{
								"patchline" : 								{
									"destination" : [ "obj-59", 0 ],
									"midpoints" : [ 257.166672706604004, 138.0, 257.500007390975952, 138.0 ],
									"source" : [ "obj-51", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-61", 2 ],
									"midpoints" : [ 212.166672706604004, 282.0, 212.50000661611557, 282.0 ],
									"source" : [ "obj-54", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-54", 2 ],
									"midpoints" : [ 257.500007390975952, 225.0, 257.766672706604027, 225.0 ],
									"source" : [ "obj-58", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-58", 0 ],
									"midpoints" : [ 257.500007390975952, 189.0, 257.500007390975952, 189.0 ],
									"source" : [ "obj-59", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-77", 0 ],
									"midpoints" : [ 120.166669964790344, 327.0, 120.166669964790344, 327.0 ],
									"source" : [ "obj-61", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-61", 0 ],
									"midpoints" : [ 120.166669964790344, 303.0, 120.166669964790344, 303.0 ],
									"source" : [ "obj-66", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-66", 0 ],
									"midpoints" : [ 90.000002086162567, 264.0, 120.166669964790344, 264.0 ],
									"source" : [ "obj-67", 1 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-76", 0 ],
									"midpoints" : [ 79.500002086162567, 162.0, 79.250002086162567, 162.0 ],
									"source" : [ "obj-67", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-59", 1 ],
									"midpoints" : [ 382.666677474975586, 153.0, 271.500007390975952, 153.0 ],
									"source" : [ "obj-71", 1 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-76", 0 ],
									"midpoints" : [ 372.166677474975586, 345.0, 79.250002086162567, 345.0 ],
									"source" : [ "obj-71", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-67", 0 ],
									"midpoints" : [ 79.500002086162567, 81.0, 79.500002086162567, 81.0 ],
									"source" : [ "obj-73", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-51", 0 ],
									"midpoints" : [ 196.166672229766846, 99.0, 257.166672706604004, 99.0 ],
									"order" : 0,
									"source" : [ "obj-74", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-54", 0 ],
									"midpoints" : [ 196.166672229766846, 243.0, 212.166672706604004, 243.0 ],
									"order" : 1,
									"source" : [ "obj-74", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-61", 1 ],
									"midpoints" : [ 196.166672229766846, 243.0, 166.333338290452957, 243.0 ],
									"order" : 2,
									"source" : [ "obj-74", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-71", 0 ],
									"midpoints" : [ 372.166677474975586, 81.0, 372.166677474975586, 81.0 ],
									"source" : [ "obj-75", 0 ]
								}

							}
 ]
					}
,
					"patching_rect" : [ 2480.0, 236.0, 51.0, 22.0 ],
					"text" : "p scaler"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-386",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 8,
					"outlettype" : [ "int", "int", "int", "int", "int", "int", "int", "int" ],
					"patching_rect" : [ 2656.0, 80.0, 95.0, 22.0 ],
					"text" : "unpack i i i i i i i i"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-387",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 2900.0, 196.0, 39.0, 22.0 ],
					"text" : "s list2"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-388",
					"maxclass" : "newobj",
					"numinlets" : 8,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 2900.0, 168.0, 92.5, 22.0 ],
					"text" : "pak i i i i i i i i"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-389",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"patching_rect" : [ 2588.0, 80.0, 58.0, 22.0 ],
					"text" : "loadbang"
				}

			}
, 			{
				"box" : 				{
					"automatic" : 1,
					"bufsize" : 48,
					"calccount" : 62,
					"id" : "obj-391",
					"maxclass" : "scope~",
					"numinlets" : 2,
					"numoutlets" : 0,
					"patching_rect" : [ 2656.0, 504.0, 128.888895034790039, 76.666670322418213 ],
					"presentation" : 1,
					"presentation_rect" : [ 406.666678786277771, 168.666671693325043, 130.0, 130.0 ]
				}

			}
, 			{
				"box" : 				{
					"candicane2" : [ 0.65098, 0.666667, 0.662745, 1.0 ],
					"candycane" : 2,
					"contdata" : 1,
					"id" : "obj-392",
					"maxclass" : "multislider",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 2376.0, 80.0, 220.0, 147.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 76.64233535528183, 151.09488970041275, 287.0, 150.0 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_invisible" : 1,
							"parameter_longname" : "multislider[2]",
							"parameter_modmode" : 0,
							"parameter_shortname" : "multislider",
							"parameter_type" : 3
						}

					}
,
					"setminmax" : [ 0.0, 1.0 ],
					"setstyle" : 1,
					"shadowblend" : 0.3,
					"shadoworientation" : 3,
					"shadowperbar" : 0,
					"size" : 8,
					"slidercolor" : [ 0.862745, 0.870588, 0.878431, 1.0 ],
					"style" : "velvet",
					"varname" : "multislider[2]"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 8.0,
					"id" : "obj-393",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 2780.0, 208.0, 27.0, 35.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 297.080290377140045, 307.299268424510956, 27.0, 35.0 ],
					"saved_attribute_attributes" : 					{
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_initial" : [ 12.0 ],
							"parameter_initial_enable" : 1,
							"parameter_longname" : "-[47]",
							"parameter_mmax" : 18.0,
							"parameter_mmin" : 1.0,
							"parameter_modmode" : 4,
							"parameter_shortname" : "1",
							"parameter_type" : 1,
							"parameter_unitstyle" : 0
						}

					}
,
					"showname" : 0,
					"textcolor" : [ 0.086275, 0.309804, 0.52549, 1.0 ],
					"varname" : "live.dial[42]"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 8.0,
					"id" : "obj-394",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 2724.0, 208.0, 27.0, 35.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 261.313867211341858, 307.299268424510956, 27.0, 35.0 ],
					"saved_attribute_attributes" : 					{
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_initial" : [ 9.0 ],
							"parameter_initial_enable" : 1,
							"parameter_longname" : "-[48]",
							"parameter_mmax" : 18.0,
							"parameter_mmin" : 1.0,
							"parameter_modmode" : 4,
							"parameter_shortname" : "1",
							"parameter_type" : 1,
							"parameter_unitstyle" : 0
						}

					}
,
					"showname" : 0,
					"textcolor" : [ 0.086275, 0.309804, 0.52549, 1.0 ],
					"varname" : "live.dial[43]"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 8.0,
					"id" : "obj-395",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 2668.0, 208.0, 27.0, 35.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 226.277371048927307, 307.299268424510956, 27.0, 35.0 ],
					"saved_attribute_attributes" : 					{
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_initial" : [ 6.0 ],
							"parameter_initial_enable" : 1,
							"parameter_longname" : "-[49]",
							"parameter_mmax" : 18.0,
							"parameter_mmin" : 1.0,
							"parameter_modmode" : 4,
							"parameter_shortname" : "1",
							"parameter_type" : 1,
							"parameter_unitstyle" : 0
						}

					}
,
					"showname" : 0,
					"textcolor" : [ 0.086275, 0.309804, 0.52549, 1.0 ],
					"varname" : "live.dial[44]"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 8.0,
					"id" : "obj-396",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 2840.0, 128.0, 27.0, 35.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 190.51094788312912, 307.299268424510956, 27.0, 35.0 ],
					"saved_attribute_attributes" : 					{
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_initial" : [ 5.0 ],
							"parameter_initial_enable" : 1,
							"parameter_longname" : "-[50]",
							"parameter_mmax" : 18.0,
							"parameter_mmin" : 1.0,
							"parameter_modmode" : 4,
							"parameter_shortname" : "1",
							"parameter_type" : 1,
							"parameter_unitstyle" : 0
						}

					}
,
					"showname" : 0,
					"textcolor" : [ 0.086275, 0.309804, 0.52549, 1.0 ],
					"varname" : "live.dial[45]"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-397",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 2780.0, 252.0, 32.0, 22.0 ],
					"text" : "7 $1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-398",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 2724.0, 252.0, 32.0, 22.0 ],
					"text" : "6 $1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-399",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 2668.0, 252.0, 32.0, 22.0 ],
					"text" : "5 $1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-400",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 2840.0, 172.0, 32.0, 22.0 ],
					"text" : "4 $1"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 8.0,
					"id" : "obj-401",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 2840.0, 208.0, 27.0, 35.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 332.846713542938232, 307.299268424510956, 27.0, 35.0 ],
					"saved_attribute_attributes" : 					{
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_initial" : [ 13.0 ],
							"parameter_initial_enable" : 1,
							"parameter_longname" : "-[51]",
							"parameter_mmax" : 18.0,
							"parameter_mmin" : 1.0,
							"parameter_modmode" : 4,
							"parameter_shortname" : "1",
							"parameter_type" : 1,
							"parameter_unitstyle" : 0
						}

					}
,
					"showname" : 0,
					"textcolor" : [ 0.086275, 0.309804, 0.52549, 1.0 ],
					"varname" : "live.dial[46]"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 8.0,
					"id" : "obj-402",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 2784.0, 128.0, 27.0, 35.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 154.744524717330933, 307.299268424510956, 27.0, 35.0 ],
					"saved_attribute_attributes" : 					{
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_initial" : [ 4.0 ],
							"parameter_initial_enable" : 1,
							"parameter_longname" : "-[52]",
							"parameter_mmax" : 18.0,
							"parameter_mmin" : 1.0,
							"parameter_modmode" : 4,
							"parameter_shortname" : "1",
							"parameter_type" : 1,
							"parameter_unitstyle" : 0
						}

					}
,
					"showname" : 0,
					"textcolor" : [ 0.086275, 0.309804, 0.52549, 1.0 ],
					"varname" : "live.dial[47]"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 8.0,
					"id" : "obj-403",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 2724.0, 128.0, 27.0, 35.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 118.978101551532745, 307.299268424510956, 27.0, 35.0 ],
					"saved_attribute_attributes" : 					{
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_initial" : [ 3.0 ],
							"parameter_initial_enable" : 1,
							"parameter_longname" : "-[53]",
							"parameter_mmax" : 18.0,
							"parameter_mmin" : 1.0,
							"parameter_modmode" : 4,
							"parameter_shortname" : "1",
							"parameter_type" : 1,
							"parameter_unitstyle" : 0
						}

					}
,
					"showname" : 0,
					"textcolor" : [ 0.086275, 0.309804, 0.52549, 1.0 ],
					"varname" : "live.dial[48]"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 8.0,
					"id" : "obj-404",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 2668.0, 128.0, 27.0, 35.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 83.941605389118195, 307.299268424510956, 27.0, 35.0 ],
					"saved_attribute_attributes" : 					{
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_initial" : [ 1.0 ],
							"parameter_initial_enable" : 1,
							"parameter_longname" : "-[54]",
							"parameter_mmax" : 18.0,
							"parameter_mmin" : 1.0,
							"parameter_modmode" : 4,
							"parameter_shortname" : "1",
							"parameter_type" : 1,
							"parameter_unitstyle" : 0
						}

					}
,
					"showname" : 0,
					"textcolor" : [ 0.086275, 0.309804, 0.52549, 1.0 ],
					"varname" : "live.dial[49]"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-405",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 2840.0, 252.0, 32.0, 22.0 ],
					"text" : "8 $1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-406",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 2784.0, 172.0, 32.0, 22.0 ],
					"text" : "3 $1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-407",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 2724.0, 172.0, 32.0, 22.0 ],
					"text" : "2 $1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-408",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 2668.0, 172.0, 32.0, 22.0 ],
					"text" : "1 $1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-409",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "multichannelsignal" ],
					"patching_rect" : [ 2656.0, 404.0, 53.0, 22.0 ],
					"text" : "mc.*~ 1."
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-410",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 3,
					"outlettype" : [ "multichannelsignal", "", "" ],
					"patching_rect" : [ 2668.0, 344.0, 111.0, 22.0 ],
					"text" : "mc.line~ @chans 8"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-411",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 2668.0, 312.0, 107.0, 22.0 ],
					"text" : "setvalue $1 $2 25."
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-412",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 2428.0, 340.0, 24.0, 24.0 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_enum" : [ "off", "on" ],
							"parameter_longname" : "button[2]",
							"parameter_mmax" : 1,
							"parameter_modmode" : 0,
							"parameter_shortname" : "button",
							"parameter_type" : 2
						}

					}
,
					"varname" : "button[2]"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-413",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 3,
					"outlettype" : [ "multichannelsignal", "", "" ],
					"patching_rect" : [ 2384.0, 516.0, 111.0, 22.0 ],
					"text" : "mc.line~ @chans 8"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-414",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 2384.0, 488.0, 107.0, 22.0 ],
					"text" : "setvalue $1 $2 25."
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-415",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 2384.0, 456.0, 47.0, 22.0 ],
					"text" : "pack i f"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-416",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 2384.0, 404.0, 47.0, 22.0 ],
					"text" : "pack f i"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-417",
					"maxclass" : "newobj",
					"numinlets" : 5,
					"numoutlets" : 4,
					"outlettype" : [ "int", "", "", "int" ],
					"patching_rect" : [ 2404.0, 376.0, 69.0, 22.0 ],
					"text" : "counter 1 8"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-418",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "float", "bang" ],
					"patching_rect" : [ 2384.0, 344.0, 29.5, 22.0 ],
					"text" : "t f b"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-419",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 2,
					"outlettype" : [ "", "" ],
					"patching_rect" : [ 2384.0, 432.0, 45.0, 22.0 ],
					"text" : "zl.rot 1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-420",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 2384.0, 316.0, 25.0, 22.0 ],
					"text" : "iter"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-421",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "multichannelsignal" ],
					"patching_rect" : [ 2388.0, 636.0, 201.0, 22.0 ],
					"text" : "mc.mixdown~ @chans 1 @autogain"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-422",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 2388.0, 544.0, 48.526148796081543, 48.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 186.0, 379.0, 48.526148796081543, 48.0 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_exponent" : 1.6,
							"parameter_initial" : [ 171.599999999999994 ],
							"parameter_initial_enable" : 1,
							"parameter_longname" : "live.dial[2]",
							"parameter_mmax" : 400.0,
							"parameter_mmin" : 40.0,
							"parameter_modmode" : 4,
							"parameter_shortname" : "live.dial",
							"parameter_type" : 0,
							"parameter_unitstyle" : 3
						}

					}
,
					"varname" : "live.dial[50]"
				}

			}
, 			{
				"box" : 				{
					"data" : 					{
						"patcher" : 						{
							"fileversion" : 1,
							"appversion" : 							{
								"major" : 9,
								"minor" : 0,
								"revision" : 7,
								"architecture" : "x64",
								"modernui" : 1
							}
,
							"classnamespace" : "dsp.gen",
							"rect" : [ 34.0, 62.0, 1402.0, 860.0 ],
							"gridsize" : [ 15.0, 15.0 ],
							"boxes" : [ 								{
									"box" : 									{
										"maxclass" : "newobj",
										"text" : "*",
										"patching_rect" : [ 318.0, 338.0, 29.5, 22.0 ],
										"numinlets" : 2,
										"numoutlets" : 1,
										"outlettype" : [ "" ],
										"id" : "obj-2"
									}

								}
, 								{
									"box" : 									{
										"maxclass" : "newobj",
										"text" : "out 3",
										"patching_rect" : [ 351.0, 385.0, 35.0, 22.0 ],
										"numinlets" : 1,
										"numoutlets" : 0,
										"id" : "obj-1"
									}

								}
, 								{
									"box" : 									{
										"maxclass" : "newobj",
										"text" : "scale -1. 1. 0.",
										"patching_rect" : [ 55.0, 236.0, 81.0, 22.0 ],
										"numinlets" : 3,
										"numoutlets" : 1,
										"outlettype" : [ "" ],
										"id" : "obj-12"
									}

								}
, 								{
									"box" : 									{
										"maxclass" : "newobj",
										"text" : "out 2",
										"patching_rect" : [ 210.360000000000014, 302.379999999999995, 35.0, 22.0 ],
										"numinlets" : 1,
										"numoutlets" : 0,
										"id" : "obj-14"
									}

								}
, 								{
									"box" : 									{
										"maxclass" : "newobj",
										"text" : "scale 0. 1. 0. 1. 1.3",
										"patching_rect" : [ 154.0, 165.0, 110.0, 22.0 ],
										"numinlets" : 1,
										"numoutlets" : 1,
										"outlettype" : [ "" ],
										"id" : "obj-13"
									}

								}
, 								{
									"box" : 									{
										"maxclass" : "newobj",
										"text" : "clip -1. 1.",
										"patching_rect" : [ 51.0, 356.0, 57.0, 22.0 ],
										"numinlets" : 1,
										"numoutlets" : 1,
										"outlettype" : [ "" ],
										"id" : "obj-4"
									}

								}
, 								{
									"box" : 									{
										"maxclass" : "newobj",
										"text" : "in 3",
										"patching_rect" : [ 562.0, 37.0, 28.0, 22.0 ],
										"numinlets" : 0,
										"numoutlets" : 1,
										"outlettype" : [ "" ],
										"id" : "obj-10"
									}

								}
, 								{
									"box" : 									{
										"maxclass" : "newobj",
										"text" : "in 2",
										"patching_rect" : [ 213.0, 128.0, 28.0, 22.0 ],
										"numinlets" : 0,
										"numoutlets" : 1,
										"outlettype" : [ "" ],
										"id" : "obj-8"
									}

								}
, 								{
									"box" : 									{
										"maxclass" : "newobj",
										"text" : "*",
										"patching_rect" : [ 50.0, 80.0, 29.5, 22.0 ],
										"numinlets" : 2,
										"numoutlets" : 1,
										"outlettype" : [ "" ],
										"id" : "obj-6"
									}

								}
, 								{
									"box" : 									{
										"maxclass" : "newobj",
										"text" : "cycle",
										"patching_rect" : [ 50.0, 115.0, 36.0, 22.0 ],
										"numinlets" : 1,
										"numoutlets" : 2,
										"outlettype" : [ "", "" ],
										"id" : "obj-5"
									}

								}
, 								{
									"box" : 									{
										"maxclass" : "newobj",
										"text" : "in 1",
										"patching_rect" : [ 50.0, 14.0, 28.0, 22.0 ],
										"numinlets" : 0,
										"numoutlets" : 1,
										"outlettype" : [ "" ],
										"id" : "obj-9"
									}

								}
, 								{
									"box" : 									{
										"maxclass" : "newobj",
										"text" : "out 1",
										"patching_rect" : [ 50.0, 414.0, 35.0, 22.0 ],
										"numinlets" : 1,
										"numoutlets" : 0,
										"id" : "obj-11"
									}

								}
 ],
							"lines" : [ 								{
									"patchline" : 									{
										"source" : [ "obj-13", 0 ],
										"destination" : [ "obj-12", 1 ],
										"order" : 2
									}

								}
, 								{
									"patchline" : 									{
										"source" : [ "obj-12", 0 ],
										"destination" : [ "obj-4", 0 ]
									}

								}
, 								{
									"patchline" : 									{
										"source" : [ "obj-5", 0 ],
										"destination" : [ "obj-12", 0 ],
										"order" : 1
									}

								}
, 								{
									"patchline" : 									{
										"source" : [ "obj-13", 0 ],
										"destination" : [ "obj-14", 0 ],
										"order" : 1
									}

								}
, 								{
									"patchline" : 									{
										"source" : [ "obj-8", 0 ],
										"destination" : [ "obj-13", 0 ]
									}

								}
, 								{
									"patchline" : 									{
										"source" : [ "obj-10", 0 ],
										"destination" : [ "obj-6", 1 ]
									}

								}
, 								{
									"patchline" : 									{
										"source" : [ "obj-6", 0 ],
										"destination" : [ "obj-5", 0 ]
									}

								}
, 								{
									"patchline" : 									{
										"source" : [ "obj-9", 0 ],
										"destination" : [ "obj-6", 0 ]
									}

								}
, 								{
									"patchline" : 									{
										"source" : [ "obj-4", 0 ],
										"destination" : [ "obj-11", 0 ]
									}

								}
, 								{
									"patchline" : 									{
										"source" : [ "obj-5", 0 ],
										"destination" : [ "obj-2", 0 ],
										"order" : 0
									}

								}
, 								{
									"patchline" : 									{
										"source" : [ "obj-2", 0 ],
										"destination" : [ "obj-1", 0 ]
									}

								}
, 								{
									"patchline" : 									{
										"source" : [ "obj-13", 0 ],
										"destination" : [ "obj-2", 1 ],
										"order" : 0
									}

								}
 ]
						}

					}
,
					"id" : "obj-423",
					"maxclass" : "newobj",
					"numinlets" : 3,
					"numoutlets" : 3,
					"outlettype" : [ "multichannelsignal", "multichannelsignal", "multichannelsignal" ],
					"patching_rect" : [ 2388.0, 604.0, 112.0, 22.0 ],
					"text" : "mc.gen~ @chans 8",
					"wrapper_uniquekey" : "u734000608"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-424",
					"maxclass" : "live.menu",
					"numinlets" : 1,
					"numoutlets" : 3,
					"outlettype" : [ "", "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 2372.0, 244.0, 100.0, 15.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 2372.0, 244.0, 100.0, 15.0 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_enum" : [ "Unscaled", "Scaled" ],
							"parameter_longname" : "live.menu[2]",
							"parameter_mmax" : 1,
							"parameter_modmode" : 0,
							"parameter_shortname" : "live.menu",
							"parameter_type" : 2
						}

					}
,
					"varname" : "live.menu[2]"
				}

			}
, 			{
				"box" : 				{
					"attr" : "active",
					"fontsize" : 8.0,
					"id" : "obj-425",
					"maxclass" : "attrui",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 2300.0, 288.0, 130.058997750282288, 17.0 ]
				}

			}
, 			{
				"box" : 				{
					"angle" : 270.0,
					"bgcolor" : [ 0.184313725490196, 0.258823529411765, 0.172549019607843, 1.0 ],
					"bordercolor" : [ 0.952941176470588, 0.8, 0.619607843137255, 1.0 ],
					"id" : "obj-426",
					"maxclass" : "panel",
					"mode" : 0,
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 2640.0, 72.0, 371.604967951774597, 302.469159960746765 ],
					"proportion" : 0.5,
					"rounded" : 19
				}

			}
, 			{
				"box" : 				{
					"angle" : 270.0,
					"bgcolor" : [ 0.184313725490196, 0.258823529411765, 0.172549019607843, 1.0 ],
					"bordercolor" : [ 0.952941176470588, 0.8, 0.619607843137255, 1.0 ],
					"id" : "obj-427",
					"maxclass" : "panel",
					"mode" : 0,
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 3044.0, 72.0, 371.604967951774597, 302.469159960746765 ],
					"proportion" : 0.5,
					"rounded" : 19
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-267",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1846.93875789642334, 24.999999523162842, 29.5, 22.0 ],
					"text" : "0."
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-268",
					"maxclass" : "newobj",
					"numinlets" : 3,
					"numoutlets" : 1,
					"outlettype" : [ "signal" ],
					"patching_rect" : [ 1218.367335319519043, 647.448973178863525, 60.0, 22.0 ],
					"text" : "clip~ 0. 1."
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-273",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 1218.367335319519043, 684.183666706085205, 45.0, 22.0 ],
					"text" : "dac~ 4"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-274",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"patching_rect" : [ 1955.102022171020508, 182.142855167388916, 29.5, 22.0 ],
					"text" : "+ 1."
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-275",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"patching_rect" : [ 2120.224888443946838, 192.134846806526184, 29.5, 22.0 ],
					"text" : "+ 1."
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-278",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"patching_rect" : [ 1897.959165573120117, 182.142855167388916, 29.5, 22.0 ],
					"text" : "+ 1."
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-279",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"patching_rect" : [ 2012.244878768920898, 186.224487781524658, 29.5, 22.0 ],
					"text" : "+ 1."
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-280",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"patching_rect" : [ 2071.42855167388916, 76.020407199859619, 29.5, 22.0 ],
					"text" : "+ 1."
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-281",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"patching_rect" : [ 2014.28569507598877, 76.020407199859619, 29.5, 22.0 ],
					"text" : "+ 1."
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-282",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"patching_rect" : [ 1957.142838478088379, 76.020407199859619, 29.5, 22.0 ],
					"text" : "+ 1."
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-283",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"patching_rect" : [ 1897.959165573120117, 71.938774585723877, 29.5, 22.0 ],
					"text" : "+ 1."
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-284",
					"maxclass" : "newobj",
					"numinlets" : 8,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 2134.693857192993164, 82.142856121063232, 92.5, 22.0 ],
					"text" : "pak i i i i i i i i"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 8.0,
					"id" : "obj-285",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 2017.244878768920898, 144.94383180141449, 27.0, 35.0 ],
					"presentation" : 1,
					"presentation_rect" : [ -26.614618403570951, 251.162781715393066, 27.0, 35.0 ],
					"saved_attribute_attributes" : 					{
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "-[23]",
							"parameter_mmax" : 0.02,
							"parameter_mmin" : -0.02,
							"parameter_modmode" : 4,
							"parameter_shortname" : "1",
							"parameter_type" : 0,
							"parameter_unitstyle" : 1
						}

					}
,
					"showname" : 0,
					"textcolor" : [ 0.086275, 0.309804, 0.52549, 1.0 ],
					"varname" : "live.dial[1]"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 8.0,
					"id" : "obj-286",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1955.102022171020508, 151.53061056137085, 27.0, 35.0 ],
					"presentation" : 1,
					"presentation_rect" : [ -67.182724680219337, 251.162781715393066, 27.0, 35.0 ],
					"saved_attribute_attributes" : 					{
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "-[29]",
							"parameter_mmax" : 0.02,
							"parameter_mmin" : -0.02,
							"parameter_modmode" : 4,
							"parameter_shortname" : "1",
							"parameter_type" : 0,
							"parameter_unitstyle" : 1
						}

					}
,
					"showname" : 0,
					"textcolor" : [ 0.086275, 0.309804, 0.52549, 1.0 ],
					"varname" : "live.dial[2]"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 8.0,
					"id" : "obj-287",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1897.959165573120117, 151.53061056137085, 27.0, 35.0 ],
					"presentation" : 1,
					"presentation_rect" : [ -107.75083095686773, 251.162781715393066, 27.0, 35.0 ],
					"saved_attribute_attributes" : 					{
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "-[31]",
							"parameter_mmax" : 0.02,
							"parameter_mmin" : -0.02,
							"parameter_modmode" : 4,
							"parameter_shortname" : "1",
							"parameter_type" : 0,
							"parameter_unitstyle" : 1
						}

					}
,
					"showname" : 0,
					"textcolor" : [ 0.086275, 0.309804, 0.52549, 1.0 ],
					"varname" : "live.dial[19]"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 8.0,
					"id" : "obj-288",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 2071.42855167388916, 39.285713672637939, 27.0, 35.0 ],
					"presentation" : 1,
					"presentation_rect" : [ -148.318937233516124, 251.162781715393066, 27.0, 35.0 ],
					"saved_attribute_attributes" : 					{
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "-[32]",
							"parameter_mmax" : 0.02,
							"parameter_mmin" : -0.02,
							"parameter_modmode" : 4,
							"parameter_shortname" : "1",
							"parameter_type" : 0,
							"parameter_unitstyle" : 1
						}

					}
,
					"showname" : 0,
					"textcolor" : [ 0.086275, 0.309804, 0.52549, 1.0 ],
					"varname" : "live.dial[20]"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-289",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 2012.244878768920898, 208.67346715927124, 32.0, 22.0 ],
					"text" : "7 $1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-290",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1955.102022171020508, 208.67346715927124, 32.0, 22.0 ],
					"text" : "6 $1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-291",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1897.959165573120117, 208.67346715927124, 32.0, 22.0 ],
					"text" : "5 $1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-292",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 2071.42855167388916, 108.673468112945557, 32.0, 22.0 ],
					"text" : "4 $1"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 8.0,
					"id" : "obj-293",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 2071.42855167388916, 151.53061056137085, 27.0, 35.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 13.953487873077435, 251.162781715393066, 27.0, 35.0 ],
					"saved_attribute_attributes" : 					{
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "-[33]",
							"parameter_mmax" : 0.02,
							"parameter_mmin" : -0.02,
							"parameter_modmode" : 4,
							"parameter_shortname" : "1",
							"parameter_type" : 0,
							"parameter_unitstyle" : 1
						}

					}
,
					"showname" : 0,
					"textcolor" : [ 0.086275, 0.309804, 0.52549, 1.0 ],
					"varname" : "live.dial[21]"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 8.0,
					"id" : "obj-294",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 2014.28569507598877, 39.285713672637939, 27.0, 35.0 ],
					"presentation" : 1,
					"presentation_rect" : [ -188.887043510164517, 251.162781715393066, 27.0, 35.0 ],
					"saved_attribute_attributes" : 					{
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "-[24]",
							"parameter_mmax" : 0.02,
							"parameter_mmin" : -0.02,
							"parameter_modmode" : 4,
							"parameter_shortname" : "1",
							"parameter_type" : 0,
							"parameter_unitstyle" : 1
						}

					}
,
					"showname" : 0,
					"textcolor" : [ 0.086275, 0.309804, 0.52549, 1.0 ],
					"varname" : "live.dial[22]"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 8.0,
					"id" : "obj-295",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1957.142838478088379, 39.285713672637939, 27.0, 35.0 ],
					"presentation" : 1,
					"presentation_rect" : [ -229.45514978681291, 251.162781715393066, 27.0, 35.0 ],
					"saved_attribute_attributes" : 					{
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "-[30]",
							"parameter_mmax" : 0.02,
							"parameter_mmin" : -0.02,
							"parameter_modmode" : 4,
							"parameter_shortname" : "1",
							"parameter_type" : 0,
							"parameter_unitstyle" : 1
						}

					}
,
					"showname" : 0,
					"textcolor" : [ 0.086275, 0.309804, 0.52549, 1.0 ],
					"varname" : "live.dial[23]"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 8.0,
					"id" : "obj-296",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1897.959165573120117, 39.285713672637939, 27.0, 35.0 ],
					"presentation" : 1,
					"presentation_rect" : [ -270.023256063461304, 250.818065285682678, 27.0, 35.0 ],
					"saved_attribute_attributes" : 					{
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "-[34]",
							"parameter_mmax" : 0.02,
							"parameter_mmin" : -0.02,
							"parameter_modmode" : 4,
							"parameter_shortname" : "1",
							"parameter_type" : 0,
							"parameter_unitstyle" : 1
						}

					}
,
					"showname" : 0,
					"textcolor" : [ 0.086275, 0.309804, 0.52549, 1.0 ],
					"varname" : "live.dial[24]"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-297",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 2079.213649213314056, 218.539343297481537, 32.0, 22.0 ],
					"text" : "8 $1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-298",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 2012.244878768920898, 108.673468112945557, 32.0, 22.0 ],
					"text" : "3 $1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-299",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1955.102022171020508, 108.673468112945557, 32.0, 22.0 ],
					"text" : "2 $1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-300",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1897.959165573120117, 108.673468112945557, 32.0, 22.0 ],
					"text" : "1 $1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-301",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 3,
					"outlettype" : [ "multichannelsignal", "", "" ],
					"patching_rect" : [ 1895.918349266052246, 296.428568363189697, 111.0, 22.0 ],
					"text" : "mc.line~ @chans 8"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-302",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1897.959165573120117, 267.857140064239502, 107.0, 22.0 ],
					"text" : "setvalue $1 $2 25."
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-303",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "int" ],
					"patching_rect" : [ 1132.653050422668457, 194.387753009796143, 33.0, 22.0 ],
					"text" : "== 0"
				}

			}
, 			{
				"box" : 				{
					"attr" : "ignoreclick",
					"fontsize" : 8.0,
					"id" : "obj-304",
					"maxclass" : "attrui",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 1132.653050422668457, 218.877548694610596, 130.058997750282288, 17.0 ]
				}

			}
, 			{
				"box" : 				{
					"activebgcolor" : [ 0.687607561603021, 0.672947940130859, 0.658307756166483, 1.0 ],
					"id" : "obj-305",
					"maxclass" : "live.numbox",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1306.1224365234375, 227.04081392288208, 44.0, 15.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 255.0, 466.0, 44.0, 15.0 ],
					"saved_attribute_attributes" : 					{
						"activebgcolor" : 						{
							"expression" : "themecolor.live_surface_bg"
						}
,
						"valueof" : 						{
							"parameter_longname" : "live.numbox[1]",
							"parameter_mmax" : 1.0,
							"parameter_modmode" : 3,
							"parameter_shortname" : "live.numbox",
							"parameter_type" : 0,
							"parameter_unitstyle" : 1
						}

					}
,
					"varname" : "live.numbox[1]"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-306",
					"maxclass" : "newobj",
					"numinlets" : 3,
					"numoutlets" : 2,
					"outlettype" : [ "bang", "" ],
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
						"rect" : [ 779.0, 62.0, 657.0, 860.0 ],
						"gridsize" : [ 15.0, 15.0 ],
						"boxes" : [ 							{
								"box" : 								{
									"id" : "obj-71",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "bang", "float" ],
									"patching_rect" : [ 362.666677474975586, 89.333335995674133, 29.5, 22.0 ],
									"text" : "t b f"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-67",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "bang", "int" ],
									"patching_rect" : [ 70.000002086162567, 138.666670799255371, 29.5, 22.0 ],
									"text" : "t b i"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-66",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "int" ],
									"patching_rect" : [ 110.666669964790344, 278.666674971580505, 29.5, 22.0 ],
									"text" : "+ 1"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-61",
									"maxclass" : "newobj",
									"numinlets" : 3,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 110.666669964790344, 304.666675746440887, 111.333336651325226, 22.0 ],
									"text" : "switch 2 1"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-59",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "float" ],
									"patching_rect" : [ 248.000007390975952, 166.666671633720398, 33.0, 22.0 ],
									"text" : "* 0.8"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-58",
									"maxclass" : "newobj",
									"numinlets" : 3,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 248.000007390975952, 200.000005960464478, 53.0, 22.0 ],
									"text" : "clip 1. 6."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-54",
									"maxclass" : "newobj",
									"numinlets" : 6,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 202.666672706604004, 258.666674375534058, 133.0, 22.0 ],
									"text" : "scale 0. 1. 0. 1."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-51",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 2,
									"outlettype" : [ "", "" ],
									"patching_rect" : [ 247.666672706604004, 114.666670083999634, 43.0, 22.0 ],
									"text" : "zl.sum"
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-73",
									"index" : 1,
									"maxclass" : "inlet",
									"numinlets" : 0,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 70.000002086162567, 49.333334803581238, 30.0, 30.0 ]
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-74",
									"index" : 2,
									"maxclass" : "inlet",
									"numinlets" : 0,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 186.666672229766846, 49.333334803581238, 30.0, 30.0 ]
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-75",
									"index" : 3,
									"maxclass" : "inlet",
									"numinlets" : 0,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 362.666677474975586, 49.333334803581238, 30.0, 30.0 ]
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-76",
									"index" : 1,
									"maxclass" : "outlet",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 69.750002086162567, 359.333344042301178, 30.0, 30.0 ]
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-77",
									"index" : 2,
									"maxclass" : "outlet",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 110.666669964790344, 359.333344042301178, 30.0, 30.0 ]
								}

							}
 ],
						"lines" : [ 							{
								"patchline" : 								{
									"destination" : [ "obj-59", 0 ],
									"midpoints" : [ 257.166672706604004, 138.0, 257.500007390975952, 138.0 ],
									"source" : [ "obj-51", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-61", 2 ],
									"midpoints" : [ 212.166672706604004, 282.0, 212.50000661611557, 282.0 ],
									"source" : [ "obj-54", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-54", 2 ],
									"midpoints" : [ 257.500007390975952, 225.0, 257.766672706604027, 225.0 ],
									"source" : [ "obj-58", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-58", 0 ],
									"midpoints" : [ 257.500007390975952, 189.0, 257.500007390975952, 189.0 ],
									"source" : [ "obj-59", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-77", 0 ],
									"midpoints" : [ 120.166669964790344, 327.0, 120.166669964790344, 327.0 ],
									"source" : [ "obj-61", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-61", 0 ],
									"midpoints" : [ 120.166669964790344, 303.0, 120.166669964790344, 303.0 ],
									"source" : [ "obj-66", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-66", 0 ],
									"midpoints" : [ 90.000002086162567, 264.0, 120.166669964790344, 264.0 ],
									"source" : [ "obj-67", 1 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-76", 0 ],
									"midpoints" : [ 79.500002086162567, 162.0, 79.250002086162567, 162.0 ],
									"source" : [ "obj-67", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-59", 1 ],
									"midpoints" : [ 382.666677474975586, 153.0, 271.500007390975952, 153.0 ],
									"source" : [ "obj-71", 1 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-76", 0 ],
									"midpoints" : [ 372.166677474975586, 345.0, 79.250002086162567, 345.0 ],
									"source" : [ "obj-71", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-67", 0 ],
									"midpoints" : [ 79.500002086162567, 81.0, 79.500002086162567, 81.0 ],
									"source" : [ "obj-73", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-51", 0 ],
									"midpoints" : [ 196.166672229766846, 99.0, 257.166672706604004, 99.0 ],
									"order" : 0,
									"source" : [ "obj-74", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-54", 0 ],
									"midpoints" : [ 196.166672229766846, 243.0, 212.166672706604004, 243.0 ],
									"order" : 1,
									"source" : [ "obj-74", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-61", 1 ],
									"midpoints" : [ 196.166672229766846, 243.0, 166.333338290452957, 243.0 ],
									"order" : 2,
									"source" : [ "obj-74", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-71", 0 ],
									"midpoints" : [ 372.166677474975586, 81.0, 372.166677474975586, 81.0 ],
									"source" : [ "obj-75", 0 ]
								}

							}
 ]
					}
,
					"patching_rect" : [ 1310.204069137573242, 192.346936702728271, 51.0, 22.0 ],
					"text" : "p scaler"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-308",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 8,
					"outlettype" : [ "int", "int", "int", "int", "int", "int", "int", "int" ],
					"patching_rect" : [ 1485.714271545410156, 35.204081058502197, 95.0, 22.0 ],
					"text" : "unpack i i i i i i i i"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-309",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 1732.653044700622559, 149.489794254302979, 39.0, 22.0 ],
					"text" : "s list2"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-310",
					"maxclass" : "newobj",
					"numinlets" : 8,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1732.653044700622559, 122.959182262420654, 92.5, 22.0 ],
					"text" : "pak i i i i i i i i"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-311",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"patching_rect" : [ 1418.36733341217041, 35.204081058502197, 58.0, 22.0 ],
					"text" : "loadbang"
				}

			}
, 			{
				"box" : 				{
					"automatic" : 1,
					"bufsize" : 48,
					"calccount" : 62,
					"id" : "obj-313",
					"maxclass" : "scope~",
					"numinlets" : 2,
					"numoutlets" : 0,
					"patching_rect" : [ 1487.755087852478027, 457.653056621551514, 128.888895034790039, 76.666670322418213 ],
					"presentation" : 1,
					"presentation_rect" : [ 404.666678726673126, 324.000009655952454, 130.0, 130.0 ]
				}

			}
, 			{
				"box" : 				{
					"candicane2" : [ 0.65098, 0.666667, 0.662745, 1.0 ],
					"candycane" : 2,
					"contdata" : 1,
					"id" : "obj-314",
					"maxclass" : "multislider",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1208.163253784179688, 35.204081058502197, 220.0, 147.0 ],
					"presentation" : 1,
					"presentation_rect" : [ -276.642334282398224, 288.321166336536407, 331.0, 155.0 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_invisible" : 1,
							"parameter_longname" : "multislider[1]",
							"parameter_modmode" : 0,
							"parameter_shortname" : "multislider",
							"parameter_type" : 3
						}

					}
,
					"setminmax" : [ 0.0, 1.0 ],
					"setstyle" : 1,
					"shadowblend" : 0.3,
					"shadowperbar" : 0,
					"size" : 8,
					"slidercolor" : [ 0.862745, 0.870588, 0.878431, 1.0 ],
					"style" : "velvet",
					"varname" : "multislider[1]"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 8.0,
					"id" : "obj-315",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1612.244882583618164, 163.775508403778076, 27.0, 35.0 ],
					"presentation" : 1,
					"presentation_rect" : [ -22.627737104892731, 444.525545060634613, 27.0, 35.0 ],
					"saved_attribute_attributes" : 					{
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "-[35]",
							"parameter_mmax" : 18.0,
							"parameter_mmin" : 1.0,
							"parameter_modmode" : 4,
							"parameter_shortname" : "1",
							"parameter_type" : 1,
							"parameter_unitstyle" : 0
						}

					}
,
					"showname" : 0,
					"textcolor" : [ 0.086275, 0.309804, 0.52549, 1.0 ],
					"varname" : "live.dial[25]"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 8.0,
					"id" : "obj-316",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1555.102025985717773, 163.775508403778076, 27.0, 35.0 ],
					"presentation" : 1,
					"presentation_rect" : [ -64.963503301143646, 444.525545060634613, 27.0, 35.0 ],
					"saved_attribute_attributes" : 					{
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "-[25]",
							"parameter_mmax" : 18.0,
							"parameter_mmin" : 1.0,
							"parameter_modmode" : 4,
							"parameter_shortname" : "1",
							"parameter_type" : 1,
							"parameter_unitstyle" : 0
						}

					}
,
					"showname" : 0,
					"textcolor" : [ 0.086275, 0.309804, 0.52549, 1.0 ],
					"varname" : "live.dial[26]"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 8.0,
					"id" : "obj-317",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1497.959169387817383, 163.775508403778076, 27.0, 35.0 ],
					"presentation" : 1,
					"presentation_rect" : [ -106.569342494010925, 444.525545060634613, 27.0, 35.0 ],
					"saved_attribute_attributes" : 					{
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "-[36]",
							"parameter_mmax" : 18.0,
							"parameter_mmin" : 1.0,
							"parameter_modmode" : 4,
							"parameter_shortname" : "1",
							"parameter_type" : 1,
							"parameter_unitstyle" : 0
						}

					}
,
					"showname" : 0,
					"textcolor" : [ 0.086275, 0.309804, 0.52549, 1.0 ],
					"varname" : "live.dial[27]"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 8.0,
					"id" : "obj-318",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1671.428555488586426, 82.142856121063232, 27.0, 35.0 ],
					"presentation" : 1,
					"presentation_rect" : [ -148.905108690261841, 444.525545060634613, 27.0, 35.0 ],
					"saved_attribute_attributes" : 					{
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "-[37]",
							"parameter_mmax" : 18.0,
							"parameter_mmin" : 1.0,
							"parameter_modmode" : 4,
							"parameter_shortname" : "1",
							"parameter_type" : 1,
							"parameter_unitstyle" : 0
						}

					}
,
					"showname" : 0,
					"textcolor" : [ 0.086275, 0.309804, 0.52549, 1.0 ],
					"varname" : "live.dial[28]"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-319",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1612.244882583618164, 206.632650852203369, 32.0, 22.0 ],
					"text" : "7 $1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-320",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1555.102025985717773, 206.632650852203369, 32.0, 22.0 ],
					"text" : "6 $1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-321",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1497.959169387817383, 206.632650852203369, 32.0, 22.0 ],
					"text" : "5 $1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-322",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1671.428555488586426, 127.040814876556396, 32.0, 22.0 ],
					"text" : "4 $1"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 8.0,
					"id" : "obj-323",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1723.595643281936646, 197.191026985645294, 27.0, 35.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 19.708029091358185, 444.525545060634613, 27.0, 35.0 ],
					"saved_attribute_attributes" : 					{
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "-[26]",
							"parameter_mmax" : 18.0,
							"parameter_mmin" : 1.0,
							"parameter_modmode" : 4,
							"parameter_shortname" : "1",
							"parameter_type" : 1,
							"parameter_unitstyle" : 0
						}

					}
,
					"showname" : 0,
					"textcolor" : [ 0.086275, 0.309804, 0.52549, 1.0 ],
					"varname" : "live.dial[29]"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 8.0,
					"id" : "obj-324",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1614.285698890686035, 82.142856121063232, 27.0, 35.0 ],
					"presentation" : 1,
					"presentation_rect" : [ -191.240874886512756, 444.525545060634613, 27.0, 35.0 ],
					"saved_attribute_attributes" : 					{
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "-[38]",
							"parameter_mmax" : 18.0,
							"parameter_mmin" : 1.0,
							"parameter_modmode" : 4,
							"parameter_shortname" : "1",
							"parameter_type" : 1,
							"parameter_unitstyle" : 0
						}

					}
,
					"showname" : 0,
					"textcolor" : [ 0.086275, 0.309804, 0.52549, 1.0 ],
					"varname" : "live.dial[30]"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 8.0,
					"id" : "obj-325",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1557.142842292785645, 82.142856121063232, 27.0, 35.0 ],
					"presentation" : 1,
					"presentation_rect" : [ -233.576641082763672, 444.525545060634613, 27.0, 35.0 ],
					"saved_attribute_attributes" : 					{
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "-[39]",
							"parameter_mmax" : 18.0,
							"parameter_mmin" : 1.0,
							"parameter_modmode" : 4,
							"parameter_shortname" : "1",
							"parameter_type" : 1,
							"parameter_unitstyle" : 0
						}

					}
,
					"showname" : 0,
					"textcolor" : [ 0.086275, 0.309804, 0.52549, 1.0 ],
					"varname" : "live.dial[31]"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 8.0,
					"id" : "obj-326",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1497.959169387817383, 82.142856121063232, 27.0, 35.0 ],
					"presentation" : 1,
					"presentation_rect" : [ -275.912407279014587, 443.795618057250977, 27.0, 35.0 ],
					"saved_attribute_attributes" : 					{
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "-[27]",
							"parameter_mmax" : 18.0,
							"parameter_mmin" : 1.0,
							"parameter_modmode" : 4,
							"parameter_shortname" : "1",
							"parameter_type" : 1,
							"parameter_unitstyle" : 0
						}

					}
,
					"showname" : 0,
					"textcolor" : [ 0.086275, 0.309804, 0.52549, 1.0 ],
					"varname" : "live.dial[32]"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-327",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1685.95519083738327, 221.191026985645294, 32.0, 22.0 ],
					"text" : "8 $1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-328",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1614.285698890686035, 127.040814876556396, 32.0, 22.0 ],
					"text" : "3 $1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-329",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1557.142842292785645, 127.040814876556396, 32.0, 22.0 ],
					"text" : "2 $1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-330",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1497.959169387817383, 127.040814876556396, 32.0, 22.0 ],
					"text" : "1 $1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-331",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "multichannelsignal" ],
					"patching_rect" : [ 1487.755087852478027, 359.693873882293701, 53.0, 22.0 ],
					"text" : "mc.*~ 1."
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-332",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 3,
					"outlettype" : [ "multichannelsignal", "", "" ],
					"patching_rect" : [ 1497.959169387817383, 298.469384670257568, 111.0, 22.0 ],
					"text" : "mc.line~ @chans 8"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-333",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1497.959169387817383, 265.816323757171631, 107.0, 22.0 ],
					"text" : "setvalue $1 $2 25."
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-334",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1259.183661460876465, 296.428568363189697, 24.0, 24.0 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_enum" : [ "off", "on" ],
							"parameter_longname" : "button[1]",
							"parameter_mmax" : 1,
							"parameter_modmode" : 0,
							"parameter_shortname" : "button",
							"parameter_type" : 2
						}

					}
,
					"varname" : "button[1]"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-335",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 3,
					"outlettype" : [ "multichannelsignal", "", "" ],
					"patching_rect" : [ 1216.326519012451172, 469.89795446395874, 111.0, 22.0 ],
					"text" : "mc.line~ @chans 8"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-336",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1216.326519012451172, 441.326526165008545, 107.0, 22.0 ],
					"text" : "setvalue $1 $2 25."
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-337",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1216.326519012451172, 412.75509786605835, 47.0, 22.0 ],
					"text" : "pack i f"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-338",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1216.326519012451172, 359.693873882293701, 47.0, 22.0 ],
					"text" : "pack f i"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-339",
					"maxclass" : "newobj",
					"numinlets" : 5,
					"numoutlets" : 4,
					"outlettype" : [ "int", "", "", "int" ],
					"patching_rect" : [ 1234.693865776062012, 329.081629276275635, 69.0, 22.0 ],
					"text" : "counter 1 8"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-340",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "float", "bang" ],
					"patching_rect" : [ 1216.326519012451172, 296.428568363189697, 29.5, 22.0 ],
					"text" : "t f b"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-341",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 2,
					"outlettype" : [ "", "" ],
					"patching_rect" : [ 1216.326519012451172, 388.265302181243896, 45.0, 22.0 ],
					"text" : "zl.rot 1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-342",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1216.326519012451172, 271.938772678375244, 25.0, 22.0 ],
					"text" : "iter"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-343",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "multichannelsignal" ],
					"patching_rect" : [ 1218.367335319519043, 590.306116580963135, 201.0, 22.0 ],
					"text" : "mc.mixdown~ @chans 1 @autogain"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-344",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1218.367335319519043, 498.469382762908936, 48.526148796081543, 48.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 255.0, 380.0, 48.526148796081543, 48.0 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_exponent" : 1.6,
							"parameter_initial" : [ 57.200000000000003 ],
							"parameter_initial_enable" : 1,
							"parameter_longname" : "live.dial[1]",
							"parameter_mmax" : 400.0,
							"parameter_mmin" : 40.0,
							"parameter_modmode" : 4,
							"parameter_shortname" : "live.dial",
							"parameter_type" : 0,
							"parameter_unitstyle" : 3
						}

					}
,
					"varname" : "live.dial[33]"
				}

			}
, 			{
				"box" : 				{
					"data" : 					{
						"patcher" : 						{
							"fileversion" : 1,
							"appversion" : 							{
								"major" : 9,
								"minor" : 0,
								"revision" : 7,
								"architecture" : "x64",
								"modernui" : 1
							}
,
							"classnamespace" : "dsp.gen",
							"rect" : [ 34.0, 62.0, 1402.0, 860.0 ],
							"gridsize" : [ 15.0, 15.0 ],
							"boxes" : [ 								{
									"box" : 									{
										"maxclass" : "newobj",
										"text" : "*",
										"patching_rect" : [ 295.0, 352.0, 29.5, 22.0 ],
										"numinlets" : 2,
										"numoutlets" : 1,
										"outlettype" : [ "" ],
										"id" : "obj-2"
									}

								}
, 								{
									"box" : 									{
										"maxclass" : "newobj",
										"text" : "out 3",
										"patching_rect" : [ 341.279999999999973, 411.769999999999982, 35.0, 22.0 ],
										"numinlets" : 1,
										"numoutlets" : 0,
										"id" : "obj-1"
									}

								}
, 								{
									"box" : 									{
										"maxclass" : "newobj",
										"text" : "scale -1. 1. 0.",
										"patching_rect" : [ 55.0, 236.0, 81.0, 22.0 ],
										"numinlets" : 3,
										"numoutlets" : 1,
										"outlettype" : [ "" ],
										"id" : "obj-12"
									}

								}
, 								{
									"box" : 									{
										"maxclass" : "newobj",
										"text" : "out 2",
										"patching_rect" : [ 210.360000000000014, 302.379999999999995, 35.0, 22.0 ],
										"numinlets" : 1,
										"numoutlets" : 0,
										"id" : "obj-14"
									}

								}
, 								{
									"box" : 									{
										"maxclass" : "newobj",
										"text" : "scale 0. 1. 0. 1. 1.3",
										"patching_rect" : [ 154.0, 165.0, 110.0, 22.0 ],
										"numinlets" : 1,
										"numoutlets" : 1,
										"outlettype" : [ "" ],
										"id" : "obj-13"
									}

								}
, 								{
									"box" : 									{
										"maxclass" : "newobj",
										"text" : "clip -1. 1.",
										"patching_rect" : [ 51.0, 356.0, 57.0, 22.0 ],
										"numinlets" : 1,
										"numoutlets" : 1,
										"outlettype" : [ "" ],
										"id" : "obj-4"
									}

								}
, 								{
									"box" : 									{
										"maxclass" : "newobj",
										"text" : "in 3",
										"patching_rect" : [ 562.0, 37.0, 28.0, 22.0 ],
										"numinlets" : 0,
										"numoutlets" : 1,
										"outlettype" : [ "" ],
										"id" : "obj-10"
									}

								}
, 								{
									"box" : 									{
										"maxclass" : "newobj",
										"text" : "in 2",
										"patching_rect" : [ 213.0, 128.0, 28.0, 22.0 ],
										"numinlets" : 0,
										"numoutlets" : 1,
										"outlettype" : [ "" ],
										"id" : "obj-8"
									}

								}
, 								{
									"box" : 									{
										"maxclass" : "newobj",
										"text" : "*",
										"patching_rect" : [ 50.0, 80.0, 29.5, 22.0 ],
										"numinlets" : 2,
										"numoutlets" : 1,
										"outlettype" : [ "" ],
										"id" : "obj-6"
									}

								}
, 								{
									"box" : 									{
										"maxclass" : "newobj",
										"text" : "cycle",
										"patching_rect" : [ 50.0, 115.0, 36.0, 22.0 ],
										"numinlets" : 1,
										"numoutlets" : 2,
										"outlettype" : [ "", "" ],
										"id" : "obj-5"
									}

								}
, 								{
									"box" : 									{
										"maxclass" : "newobj",
										"text" : "in 1",
										"patching_rect" : [ 50.0, 14.0, 28.0, 22.0 ],
										"numinlets" : 0,
										"numoutlets" : 1,
										"outlettype" : [ "" ],
										"id" : "obj-9"
									}

								}
, 								{
									"box" : 									{
										"maxclass" : "newobj",
										"text" : "out 1",
										"patching_rect" : [ 50.0, 414.0, 35.0, 22.0 ],
										"numinlets" : 1,
										"numoutlets" : 0,
										"id" : "obj-11"
									}

								}
 ],
							"lines" : [ 								{
									"patchline" : 									{
										"source" : [ "obj-13", 0 ],
										"destination" : [ "obj-12", 1 ],
										"order" : 2
									}

								}
, 								{
									"patchline" : 									{
										"source" : [ "obj-12", 0 ],
										"destination" : [ "obj-4", 0 ]
									}

								}
, 								{
									"patchline" : 									{
										"source" : [ "obj-5", 0 ],
										"destination" : [ "obj-12", 0 ],
										"order" : 1
									}

								}
, 								{
									"patchline" : 									{
										"source" : [ "obj-13", 0 ],
										"destination" : [ "obj-14", 0 ],
										"order" : 1
									}

								}
, 								{
									"patchline" : 									{
										"source" : [ "obj-8", 0 ],
										"destination" : [ "obj-13", 0 ]
									}

								}
, 								{
									"patchline" : 									{
										"source" : [ "obj-10", 0 ],
										"destination" : [ "obj-6", 1 ]
									}

								}
, 								{
									"patchline" : 									{
										"source" : [ "obj-6", 0 ],
										"destination" : [ "obj-5", 0 ]
									}

								}
, 								{
									"patchline" : 									{
										"source" : [ "obj-9", 0 ],
										"destination" : [ "obj-6", 0 ]
									}

								}
, 								{
									"patchline" : 									{
										"source" : [ "obj-4", 0 ],
										"destination" : [ "obj-11", 0 ]
									}

								}
, 								{
									"patchline" : 									{
										"source" : [ "obj-5", 0 ],
										"destination" : [ "obj-2", 0 ],
										"order" : 0
									}

								}
, 								{
									"patchline" : 									{
										"source" : [ "obj-2", 0 ],
										"destination" : [ "obj-1", 0 ]
									}

								}
, 								{
									"patchline" : 									{
										"source" : [ "obj-13", 0 ],
										"destination" : [ "obj-2", 1 ],
										"order" : 0
									}

								}
 ]
						}

					}
,
					"id" : "obj-345",
					"maxclass" : "newobj",
					"numinlets" : 3,
					"numoutlets" : 3,
					"outlettype" : [ "multichannelsignal", "multichannelsignal", "multichannelsignal" ],
					"patching_rect" : [ 1218.367335319519043, 561.734688282012939, 112.0, 22.0 ],
					"text" : "mc.gen~ @chans 8",
					"wrapper_uniquekey" : "u146000700"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-346",
					"maxclass" : "live.menu",
					"numinlets" : 1,
					"numoutlets" : 3,
					"outlettype" : [ "", "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1202.040804862976074, 198.469385623931885, 100.0, 15.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 476.033031463623047, 759.504090130329132, 100.0, 15.0 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_enum" : [ "Unscaled", "Scaled" ],
							"parameter_longname" : "live.menu[1]",
							"parameter_mmax" : 1,
							"parameter_modmode" : 0,
							"parameter_shortname" : "live.menu",
							"parameter_type" : 2
						}

					}
,
					"varname" : "live.menu[1]"
				}

			}
, 			{
				"box" : 				{
					"attr" : "active",
					"fontsize" : 8.0,
					"id" : "obj-347",
					"maxclass" : "attrui",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 1132.653050422668457, 243.367344379425049, 130.058997750282288, 17.0 ]
				}

			}
, 			{
				"box" : 				{
					"angle" : 270.0,
					"bgcolor" : [ 0.184313725490196, 0.258823529411765, 0.172549019607843, 1.0 ],
					"bordercolor" : [ 0.952941176470588, 0.8, 0.619607843137255, 1.0 ],
					"id" : "obj-348",
					"maxclass" : "panel",
					"mode" : 0,
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 1469.387741088867188, 24.999999523162842, 371.604967951774597, 302.469159960746765 ],
					"proportion" : 0.5,
					"rounded" : 19
				}

			}
, 			{
				"box" : 				{
					"angle" : 270.0,
					"bgcolor" : [ 0.184313725490196, 0.258823529411765, 0.172549019607843, 1.0 ],
					"bordercolor" : [ 0.952941176470588, 0.8, 0.619607843137255, 1.0 ],
					"id" : "obj-349",
					"maxclass" : "panel",
					"mode" : 0,
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 1873.469369888305664, 24.999999523162842, 371.604967951774597, 302.469159960746765 ],
					"proportion" : 0.5,
					"rounded" : 19
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-33",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 716.176456928253174, 16.176470279693604, 29.5, 22.0 ],
					"text" : "0."
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-28",
					"maxclass" : "newobj",
					"numinlets" : 3,
					"numoutlets" : 1,
					"outlettype" : [ "signal" ],
					"patching_rect" : [ 87.755101203918457, 619.0, 60.0, 22.0 ],
					"text" : "clip~ 0. 1."
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-10",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 87.755101203918457, 733.0, 45.0, 22.0 ],
					"text" : "dac~ 3"
				}

			}
, 			{
				"box" : 				{
					"code" : "{\n\t\"cols\" : 2,\n\t\"data\" : \t{\n\t\t\"point-20\" : [ 0.417322844266891, 0.448818892240524 ],\n\t\t\"point-21\" : [ 0.118110239505768, 0.125984251499176 ],\n\t\t\"point-22\" : [ 0.149606302380562, 0.834645688533783 ],\n\t\t\"point-23\" : [ 0.582677185535431, 0.496062994003296 ],\n\t\t\"point-24\" : [ 0.771653532981873, 0.93700784444809 ],\n\t\t\"point-25\" : [ 0.811023592948914, 0.086614176630974 ]\n\t}\n\n}\n",
					"fontface" : 0,
					"fontname" : "<Monospaced>",
					"fontsize" : 12.0,
					"id" : "obj-271",
					"maxclass" : "dict.codebox",
					"numinlets" : 2,
					"numoutlets" : 5,
					"outlettype" : [ "dictionary", "", "", "", "" ],
					"patching_rect" : [ 1152.380941390991211, 1914.285696029663086, 340.0, 200.0 ],
					"saved_object_attributes" : 					{
						"legacy" : 0,
						"parameter_enable" : 0,
						"parameter_mappable" : 0
					}

				}

			}
, 			{
				"box" : 				{
					"id" : "obj-269",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1533.333318710327148, 1857.142839431762695, 39.0, 22.0 ],
					"text" : "dump"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-244",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1533.333318710327148, 1642.85712718963623, 39.0, 22.0 ],
					"text" : "dump"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-16",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 2,
					"outlettype" : [ "", "" ],
					"patching_rect" : [ 1419.047605514526367, 1847.619029998779297, 74.0, 22.0 ],
					"text" : "route dump"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-237",
					"maxclass" : "live.text",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1195.238083839416504, 1252.380940437316895, 44.0, 15.0 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_enum" : [ "val1", "val2" ],
							"parameter_longname" : "live.text",
							"parameter_mmax" : 1,
							"parameter_modmode" : 0,
							"parameter_shortname" : "live.text",
							"parameter_type" : 2
						}

					}
,
					"text" : "pause",
					"texton" : "resume",
					"varname" : "live.text"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-219",
					"maxclass" : "number",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 961.904752731323242, 1261.904749870300293, 50.0, 22.0 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_invisible" : 1,
							"parameter_longname" : "number",
							"parameter_modmode" : 0,
							"parameter_shortname" : "number",
							"parameter_type" : 3
						}

					}
,
					"varname" : "number"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-216",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 933.333324432373047, 1457.142843246459961, 36.0, 22.0 ],
					"text" : "defer"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-215",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 933.333324432373047, 1399.99998664855957, 29.5, 22.0 ],
					"text" : "join"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-214",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 933.333324432373047, 1433.333319664001465, 39.0, 22.0 ],
					"text" : "$1 $2"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-197",
					"maxclass" : "newobj",
					"numinlets" : 6,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1099.99998950958252, 1376.190463066101074, 97.0, 22.0 ],
					"text" : "scale 0. 1. 0 127"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-196",
					"maxclass" : "newobj",
					"numinlets" : 6,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 933.333324432373047, 1371.428558349609375, 97.0, 22.0 ],
					"text" : "scale 0. 1. 0 127"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-192",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 937.777822494506836, 1262.222282409667969, 24.0, 24.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 764.17162744078064, 673.904065310955048, 10.958903312683105, 10.958903312683105 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_enum" : [ "off", "on" ],
							"parameter_longname" : "button[6]",
							"parameter_mmax" : 1,
							"parameter_modmode" : 0,
							"parameter_shortname" : "button[6]",
							"parameter_type" : 2
						}

					}
,
					"varname" : "button[6]"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-190",
					"maxclass" : "newobj",
					"numinlets" : 3,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"patching_rect" : [ 1099.99998950958252, 1347.619034767150879, 44.0, 22.0 ],
					"text" : "line 50"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-188",
					"linecount" : 3,
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1099.99998950958252, 1290.476178169250488, 140.404033541679382, 49.0 ],
					"text" : "0.44882 $1 0.12598 $1 0.83465 $1 0.49606 $1 0.93701 $1 0.086614 $1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-187",
					"maxclass" : "newobj",
					"numinlets" : 3,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"patching_rect" : [ 933.333324432373047, 1347.619034767150879, 44.0, 22.0 ],
					"text" : "line 50"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-186",
					"linecount" : 3,
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 933.333324432373047, 1290.476178169250488, 138.961037635803223, 49.0 ],
					"text" : "0.41732 $1 0.11811 $1 0.14961 $1 0.58268 $1 0.77165 $1 0.81102 $1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-170",
					"linecount" : 2,
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1361.904748916625977, 1766.66664981842041, 50.0, 35.0 ],
					"text" : "addpoint"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-145",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"patching_rect" : [ 823.529396057128906, 173.52940845489502, 29.5, 22.0 ],
					"text" : "+ 1."
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-143",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"patching_rect" : [ 976.404572367668152, 176.470584869384766, 29.5, 22.0 ],
					"text" : "+ 1."
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-141",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"patching_rect" : [ 767.64704418182373, 173.52940845489502, 29.5, 22.0 ],
					"text" : "+ 1."
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-140",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"patching_rect" : [ 882.352924346923828, 176.470584869384766, 29.5, 22.0 ],
					"text" : "+ 1."
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-135",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"patching_rect" : [ 941.17645263671875, 67.64705753326416, 29.5, 22.0 ],
					"text" : "+ 1."
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-117",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"patching_rect" : [ 883.823512554168701, 66.176469326019287, 29.5, 22.0 ],
					"text" : "+ 1."
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-116",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"patching_rect" : [ 826.470572471618652, 66.176469326019287, 29.5, 22.0 ],
					"text" : "+ 1."
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-115",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "float" ],
					"patching_rect" : [ 767.64704418182373, 63.235292911529541, 29.5, 22.0 ],
					"text" : "+ 1."
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-95",
					"maxclass" : "newobj",
					"numinlets" : 8,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1002.941157341003418, 72.058822154998779, 92.5, 22.0 ],
					"text" : "pak i i i i i i i i"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 8.0,
					"id" : "obj-96",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 882.352924346923828, 142.647056102752686, 27.0, 35.0 ],
					"presentation" : 1,
					"presentation_rect" : [ -26.78073135444096, 9.302325248718262, 27.0, 35.0 ],
					"saved_attribute_attributes" : 					{
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "-[8]",
							"parameter_mmax" : 0.02,
							"parameter_mmin" : -0.02,
							"parameter_modmode" : 4,
							"parameter_shortname" : "1",
							"parameter_type" : 0,
							"parameter_unitstyle" : 1
						}

					}
,
					"showname" : 0,
					"textcolor" : [ 0.086275, 0.309804, 0.52549, 1.0 ],
					"varname" : "live.dial[11]"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 8.0,
					"id" : "obj-97",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 823.529396057128906, 142.647056102752686, 27.0, 35.0 ],
					"presentation" : 1,
					"presentation_rect" : [ -67.514950581959312, 9.302325248718262, 27.0, 35.0 ],
					"saved_attribute_attributes" : 					{
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "-[9]",
							"parameter_mmax" : 0.02,
							"parameter_mmin" : -0.02,
							"parameter_modmode" : 4,
							"parameter_shortname" : "1",
							"parameter_type" : 0,
							"parameter_unitstyle" : 1
						}

					}
,
					"showname" : 0,
					"textcolor" : [ 0.086275, 0.309804, 0.52549, 1.0 ],
					"varname" : "live.dial[12]"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 8.0,
					"id" : "obj-98",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 767.64704418182373, 142.647056102752686, 27.0, 35.0 ],
					"presentation" : 1,
					"presentation_rect" : [ -108.249169809477664, 9.302325248718262, 27.0, 35.0 ],
					"saved_attribute_attributes" : 					{
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "-[10]",
							"parameter_mmax" : 0.02,
							"parameter_mmin" : -0.02,
							"parameter_modmode" : 4,
							"parameter_shortname" : "1",
							"parameter_type" : 0,
							"parameter_unitstyle" : 1
						}

					}
,
					"showname" : 0,
					"textcolor" : [ 0.086275, 0.309804, 0.52549, 1.0 ],
					"varname" : "live.dial[13]"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 8.0,
					"id" : "obj-99",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 941.17645263671875, 30.882352352142334, 27.0, 35.0 ],
					"presentation" : 1,
					"presentation_rect" : [ -148.983389036996016, 9.302325248718262, 27.0, 35.0 ],
					"saved_attribute_attributes" : 					{
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "-[11]",
							"parameter_mmax" : 0.02,
							"parameter_mmin" : -0.02,
							"parameter_modmode" : 4,
							"parameter_shortname" : "1",
							"parameter_type" : 0,
							"parameter_unitstyle" : 1
						}

					}
,
					"showname" : 0,
					"textcolor" : [ 0.086275, 0.309804, 0.52549, 1.0 ],
					"varname" : "live.dial[14]"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-100",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 882.352924346923828, 199.999996185302734, 32.0, 22.0 ],
					"text" : "7 $1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-101",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 823.529396057128906, 199.999996185302734, 32.0, 22.0 ],
					"text" : "6 $1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-102",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 767.64704418182373, 199.999996185302734, 32.0, 22.0 ],
					"text" : "5 $1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-103",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 941.17645263671875, 98.529409885406494, 32.0, 22.0 ],
					"text" : "4 $1"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 8.0,
					"id" : "obj-104",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 941.17645263671875, 142.647056102752686, 27.0, 35.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 13.953487873077393, 9.302325248718262, 27.0, 35.0 ],
					"saved_attribute_attributes" : 					{
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "-[12]",
							"parameter_mmax" : 0.02,
							"parameter_mmin" : -0.02,
							"parameter_modmode" : 4,
							"parameter_shortname" : "1",
							"parameter_type" : 0,
							"parameter_unitstyle" : 1
						}

					}
,
					"showname" : 0,
					"textcolor" : [ 0.086275, 0.309804, 0.52549, 1.0 ],
					"varname" : "live.dial[15]"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 8.0,
					"id" : "obj-105",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 883.823512554168701, 30.882352352142334, 27.0, 35.0 ],
					"presentation" : 1,
					"presentation_rect" : [ -189.717608264514354, 9.302325248718262, 27.0, 35.0 ],
					"saved_attribute_attributes" : 					{
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "-[13]",
							"parameter_mmax" : 0.02,
							"parameter_mmin" : -0.02,
							"parameter_modmode" : 4,
							"parameter_shortname" : "1",
							"parameter_type" : 0,
							"parameter_unitstyle" : 1
						}

					}
,
					"showname" : 0,
					"textcolor" : [ 0.086275, 0.309804, 0.52549, 1.0 ],
					"varname" : "live.dial[16]"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 8.0,
					"id" : "obj-106",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 826.470572471618652, 30.882352352142334, 27.0, 35.0 ],
					"presentation" : 1,
					"presentation_rect" : [ -230.45182749203272, 9.302325248718262, 27.0, 35.0 ],
					"saved_attribute_attributes" : 					{
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "-[14]",
							"parameter_mmax" : 0.02,
							"parameter_mmin" : -0.02,
							"parameter_modmode" : 4,
							"parameter_shortname" : "1",
							"parameter_type" : 0,
							"parameter_unitstyle" : 1
						}

					}
,
					"showname" : 0,
					"textcolor" : [ 0.086275, 0.309804, 0.52549, 1.0 ],
					"varname" : "live.dial[17]"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 8.0,
					"id" : "obj-107",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 767.64704418182373, 30.882352352142334, 27.0, 35.0 ],
					"presentation" : 1,
					"presentation_rect" : [ -271.186046719551086, 8.957608819007874, 27.0, 35.0 ],
					"saved_attribute_attributes" : 					{
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "-[15]",
							"parameter_mmax" : 0.02,
							"parameter_mmin" : -0.02,
							"parameter_modmode" : 4,
							"parameter_shortname" : "1",
							"parameter_type" : 0,
							"parameter_unitstyle" : 1
						}

					}
,
					"showname" : 0,
					"textcolor" : [ 0.086275, 0.309804, 0.52549, 1.0 ],
					"varname" : "live.dial[18]"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-108",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 941.17645263671875, 199.999996185302734, 32.0, 22.0 ],
					"text" : "8 $1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-109",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 882.352924346923828, 98.529409885406494, 32.0, 22.0 ],
					"text" : "3 $1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-111",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 824.999984264373779, 98.529409885406494, 32.0, 22.0 ],
					"text" : "2 $1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-112",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 766.176455974578857, 98.529409885406494, 32.0, 22.0 ],
					"text" : "1 $1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-113",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 3,
					"outlettype" : [ "multichannelsignal", "", "" ],
					"patching_rect" : [ 764.705867767333984, 286.764700412750244, 111.0, 22.0 ],
					"text" : "mc.line~ @chans 8"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-114",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 767.64704418182373, 258.823524475097656, 107.0, 22.0 ],
					"text" : "setvalue $1 $2 25."
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-87",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "int" ],
					"patching_rect" : [ 1.472533702850342, 184.59375, 33.0, 22.0 ],
					"text" : "== 0"
				}

			}
, 			{
				"box" : 				{
					"attr" : "ignoreclick",
					"fontsize" : 8.0,
					"id" : "obj-85",
					"maxclass" : "attrui",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 1.472533702850342, 210.375, 130.058997750282288, 17.0 ]
				}

			}
, 			{
				"box" : 				{
					"activebgcolor" : [ 0.687607561603021, 0.672947940130859, 0.658307756166483, 1.0 ],
					"id" : "obj-79",
					"maxclass" : "live.numbox",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 175.0, 217.375, 44.0, 15.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 114.0, 464.0, 44.0, 15.0 ],
					"saved_attribute_attributes" : 					{
						"activebgcolor" : 						{
							"expression" : "themecolor.live_surface_bg"
						}
,
						"valueof" : 						{
							"parameter_longname" : "live.numbox",
							"parameter_mmax" : 1.0,
							"parameter_modmode" : 3,
							"parameter_shortname" : "live.numbox",
							"parameter_type" : 0,
							"parameter_unitstyle" : 1
						}

					}
,
					"varname" : "live.numbox"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-78",
					"maxclass" : "newobj",
					"numinlets" : 3,
					"numoutlets" : 2,
					"outlettype" : [ "bang", "" ],
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
						"rect" : [ 34.0, 62.0, 1402.0, 860.0 ],
						"gridsize" : [ 15.0, 15.0 ],
						"boxes" : [ 							{
								"box" : 								{
									"id" : "obj-71",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "bang", "float" ],
									"patching_rect" : [ 362.666677474975586, 89.333335995674133, 29.5, 22.0 ],
									"text" : "t b f"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-67",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "bang", "int" ],
									"patching_rect" : [ 70.000002086162567, 138.666670799255371, 29.5, 22.0 ],
									"text" : "t b i"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-66",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "int" ],
									"patching_rect" : [ 110.666669964790344, 278.666674971580505, 29.5, 22.0 ],
									"text" : "+ 1"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-61",
									"maxclass" : "newobj",
									"numinlets" : 3,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 110.666669964790344, 304.666675746440887, 111.333336651325226, 22.0 ],
									"text" : "switch 2 1"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-59",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "float" ],
									"patching_rect" : [ 248.000007390975952, 166.666671633720398, 33.0, 22.0 ],
									"text" : "* 0.8"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-58",
									"maxclass" : "newobj",
									"numinlets" : 3,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 248.000007390975952, 200.000005960464478, 53.0, 22.0 ],
									"text" : "clip 1. 6."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-54",
									"maxclass" : "newobj",
									"numinlets" : 6,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 202.666672706604004, 258.666674375534058, 133.0, 22.0 ],
									"text" : "scale 0. 1. 0. 1."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-51",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 2,
									"outlettype" : [ "", "" ],
									"patching_rect" : [ 247.666672706604004, 114.666670083999634, 43.0, 22.0 ],
									"text" : "zl.sum"
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-73",
									"index" : 1,
									"maxclass" : "inlet",
									"numinlets" : 0,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 70.000002086162567, 49.333334803581238, 30.0, 30.0 ]
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-74",
									"index" : 2,
									"maxclass" : "inlet",
									"numinlets" : 0,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 186.666672229766846, 49.333334803581238, 30.0, 30.0 ]
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-75",
									"index" : 3,
									"maxclass" : "inlet",
									"numinlets" : 0,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 362.666677474975586, 49.333334803581238, 30.0, 30.0 ]
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-76",
									"index" : 1,
									"maxclass" : "outlet",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 69.750002086162567, 359.333344042301178, 30.0, 30.0 ]
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-77",
									"index" : 2,
									"maxclass" : "outlet",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 110.666669964790344, 359.333344042301178, 30.0, 30.0 ]
								}

							}
 ],
						"lines" : [ 							{
								"patchline" : 								{
									"destination" : [ "obj-59", 0 ],
									"midpoints" : [ 257.166672706604004, 138.0, 257.500007390975952, 138.0 ],
									"source" : [ "obj-51", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-61", 2 ],
									"midpoints" : [ 212.166672706604004, 282.0, 212.50000661611557, 282.0 ],
									"source" : [ "obj-54", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-54", 2 ],
									"midpoints" : [ 257.500007390975952, 225.0, 257.766672706604027, 225.0 ],
									"source" : [ "obj-58", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-58", 0 ],
									"midpoints" : [ 257.500007390975952, 189.0, 257.500007390975952, 189.0 ],
									"source" : [ "obj-59", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-77", 0 ],
									"midpoints" : [ 120.166669964790344, 327.0, 120.166669964790344, 327.0 ],
									"source" : [ "obj-61", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-61", 0 ],
									"midpoints" : [ 120.166669964790344, 303.0, 120.166669964790344, 303.0 ],
									"source" : [ "obj-66", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-66", 0 ],
									"midpoints" : [ 90.000002086162567, 264.0, 120.166669964790344, 264.0 ],
									"source" : [ "obj-67", 1 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-76", 0 ],
									"midpoints" : [ 79.500002086162567, 162.0, 79.250002086162567, 162.0 ],
									"source" : [ "obj-67", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-59", 1 ],
									"midpoints" : [ 382.666677474975586, 153.0, 271.500007390975952, 153.0 ],
									"source" : [ "obj-71", 1 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-76", 0 ],
									"midpoints" : [ 372.166677474975586, 345.0, 79.250002086162567, 345.0 ],
									"source" : [ "obj-71", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-67", 0 ],
									"midpoints" : [ 79.500002086162567, 81.0, 79.500002086162567, 81.0 ],
									"source" : [ "obj-73", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-51", 0 ],
									"midpoints" : [ 196.166672229766846, 99.0, 257.166672706604004, 99.0 ],
									"order" : 0,
									"source" : [ "obj-74", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-54", 0 ],
									"midpoints" : [ 196.166672229766846, 243.0, 212.166672706604004, 243.0 ],
									"order" : 1,
									"source" : [ "obj-74", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-61", 1 ],
									"midpoints" : [ 196.166672229766846, 243.0, 166.333338290452957, 243.0 ],
									"order" : 2,
									"source" : [ "obj-74", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-71", 0 ],
									"midpoints" : [ 372.166677474975586, 81.0, 372.166677474975586, 81.0 ],
									"source" : [ "obj-75", 0 ]
								}

							}
 ]
					}
,
					"patching_rect" : [ 179.6875, 182.8125, 51.0, 22.0 ],
					"text" : "p scaler"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-45",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1180.952369689941406, 1795.238078117370605, 37.0, 22.0 ],
					"text" : "r list1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-44",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 235.9375, 182.8125, 39.0, 22.0 ],
					"text" : "s list1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-43",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 8,
					"outlettype" : [ "int", "int", "int", "int", "int", "int", "int", "int" ],
					"patching_rect" : [ 355.114550650119781, 24.999999523162842, 95.0, 22.0 ],
					"text" : "unpack i i i i i i i i"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-29",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 602.173369467258453, 141.176467895507812, 39.0, 22.0 ],
					"text" : "s list2"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-2",
					"maxclass" : "newobj",
					"numinlets" : 8,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 602.173369467258453, 113.235291957855225, 92.5, 22.0 ],
					"text" : "pak i i i i i i i i"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-12",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"patching_rect" : [ 288.235288619995117, 24.999999523162842, 58.0, 22.0 ],
					"text" : "loadbang"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-25",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 78.125, -0.884955823421478, 87.0, 22.0 ],
					"text" : "r paramsbuflist"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 13.0,
					"id" : "obj-84",
					"maxclass" : "comment",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 790.47618293762207, 1719.047602653503418, 109.0, 21.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 1079.999897003173828, 720.716086030006409, 109.0, 21.0 ],
					"text" : "predicting points",
					"textcolor" : [ 0.992156862745098, 0.941176470588235, 0.835294117647059, 1.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-136",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "float", "bang" ],
					"patching_rect" : [ 1180.952369689941406, 1752.380935668945312, 167.0, 22.0 ],
					"text" : "buffer~ paramsbuf @samps 8"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-110",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 509.523804664611816, 1533.333318710327148, 22.0, 22.0 ],
					"text" : "r y"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-193",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 485.71428108215332, 1533.333318710327148, 22.0, 22.0 ],
					"text" : "r x"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-265",
					"maxclass" : "newobj",
					"numinlets" : 4,
					"numoutlets" : 12,
					"outlettype" : [ "", "", "", "", "", "", "", "", "", "", "", "" ],
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
						"rect" : [ 34.0, 62.0, 1402.0, 860.0 ],
						"gridsize" : [ 15.0, 15.0 ],
						"boxes" : [ 							{
								"box" : 								{
									"comment" : "ry in",
									"id" : "obj-3",
									"index" : 4,
									"maxclass" : "inlet",
									"numinlets" : 0,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 607.462664842605591, 20.895521640777588, 30.0, 30.0 ]
								}

							}
, 							{
								"box" : 								{
									"comment" : "r x in",
									"id" : "obj-2",
									"index" : 3,
									"maxclass" : "inlet",
									"numinlets" : 0,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 442.746252536773682, 40.0, 30.0, 30.0 ]
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-226",
									"maxclass" : "newobj",
									"numinlets" : 0,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 212.5, 311.0, 36.0, 22.0 ],
									"text" : "r add"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-168",
									"maxclass" : "message",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 65.5, 346.0, 29.5, 22.0 ],
									"text" : "0"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-172",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 6,
									"outlettype" : [ "", "", "", "", "", "" ],
									"patching_rect" : [ 281.5, 539.0, 71.5, 22.0 ],
									"text" : "gate 6 1"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-174",
									"maxclass" : "newobj",
									"numinlets" : 5,
									"numoutlets" : 4,
									"outlettype" : [ "int", "", "", "int" ],
									"patching_rect" : [ 272.5, 365.0, 69.0, 22.0 ],
									"text" : "counter 1 6"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-184",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 6,
									"outlettype" : [ "", "", "", "", "", "" ],
									"patching_rect" : [ 65.5, 370.0, 71.5, 22.0 ],
									"text" : "gate 6 1"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-185",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 4,
									"outlettype" : [ "bang", "bang", "bang", "bang" ],
									"patching_rect" : [ 212.5, 365.0, 52.0, 22.0 ],
									"text" : "t b b b b"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-186",
									"maxclass" : "button",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "bang" ],
									"parameter_enable" : 0,
									"patching_rect" : [ 212.5, 338.0, 24.0, 24.0 ]
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-187",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "int" ],
									"patching_rect" : [ 194.5, 208.0, 29.5, 22.0 ],
									"text" : "- 16"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-188",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "float" ],
									"patching_rect" : [ 64.5, 208.0, 33.0, 22.0 ],
									"text" : "- 16."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-190",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "int" ],
									"patching_rect" : [ 194.5, 263.0, 29.5, 22.0 ],
									"text" : "+"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-191",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "int" ],
									"patching_rect" : [ 65.5, 263.0, 29.5, 22.0 ],
									"text" : "+"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-192",
									"maxclass" : "newobj",
									"numinlets" : 6,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 167.5, 166.0, 97.0, 22.0 ],
									"text" : "scale 0 127 1. 0."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-194",
									"maxclass" : "newobj",
									"numinlets" : 6,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 64.5, 166.0, 97.0, 22.0 ],
									"text" : "scale 0 127 0. 1."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-196",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 114.5, 289.0, 47.0, 22.0 ],
									"text" : "pack f f"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-197",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "float" ],
									"patching_rect" : [ 194.5, 236.0, 29.5, 22.0 ],
									"text" : "* 0."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-198",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "float" ],
									"patching_rect" : [ 65.5, 236.0, 29.5, 22.0 ],
									"text" : "* 0."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-201",
									"maxclass" : "button",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "bang" ],
									"parameter_enable" : 0,
									"patching_rect" : [ 126.0, 207.0, 24.0, 24.0 ]
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-202",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 4,
									"outlettype" : [ "float", "float", "float", "float" ],
									"patching_rect" : [ 109.5, 76.0, 74.0, 22.0 ],
									"text" : "unpack f f f f"
								}

							}
, 							{
								"box" : 								{
									"comment" : "clear",
									"id" : "obj-227",
									"index" : 2,
									"maxclass" : "inlet",
									"numinlets" : 0,
									"numoutlets" : 1,
									"outlettype" : [ "bang" ],
									"patching_rect" : [ 361.0, 33.0, 30.0, 30.0 ]
								}

							}
, 							{
								"box" : 								{
									"comment" : "grid position",
									"id" : "obj-228",
									"index" : 1,
									"maxclass" : "inlet",
									"numinlets" : 0,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 109.5, 40.0, 30.0, 30.0 ]
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-233",
									"index" : 1,
									"maxclass" : "outlet",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 65.5, 621.0, 30.0, 30.0 ]
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-244",
									"index" : 2,
									"maxclass" : "outlet",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 117.5, 621.0, 30.0, 30.0 ]
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-247",
									"index" : 3,
									"maxclass" : "outlet",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 152.5, 621.0, 30.0, 30.0 ]
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-251",
									"index" : 4,
									"maxclass" : "outlet",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 187.5, 621.0, 30.0, 30.0 ]
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-257",
									"index" : 5,
									"maxclass" : "outlet",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 222.5, 621.0, 30.0, 30.0 ]
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-258",
									"index" : 6,
									"maxclass" : "outlet",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 257.5, 621.0, 30.0, 30.0 ]
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-259",
									"index" : 7,
									"maxclass" : "outlet",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 292.5, 621.0, 30.0, 30.0 ]
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-260",
									"index" : 8,
									"maxclass" : "outlet",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 333.5, 621.0, 30.0, 30.0 ]
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-261",
									"index" : 9,
									"maxclass" : "outlet",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 368.5, 621.0, 30.0, 30.0 ]
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-262",
									"index" : 10,
									"maxclass" : "outlet",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 403.5, 621.0, 30.0, 30.0 ]
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-263",
									"index" : 11,
									"maxclass" : "outlet",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 438.5, 621.0, 30.0, 30.0 ]
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-264",
									"index" : 12,
									"maxclass" : "outlet",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 473.5, 621.0, 30.0, 30.0 ]
								}

							}
 ],
						"lines" : [ 							{
								"patchline" : 								{
									"destination" : [ "obj-184", 0 ],
									"midpoints" : [ 75.0, 356.432215571403503, 75.0, 356.432215571403503 ],
									"source" : [ "obj-168", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-259", 0 ],
									"source" : [ "obj-172", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-260", 0 ],
									"source" : [ "obj-172", 5 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-261", 0 ],
									"source" : [ "obj-172", 4 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-262", 0 ],
									"source" : [ "obj-172", 3 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-263", 0 ],
									"source" : [ "obj-172", 2 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-264", 0 ],
									"source" : [ "obj-172", 1 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-172", 0 ],
									"midpoints" : [ 282.0, 526.0, 291.0, 526.0 ],
									"order" : 0,
									"source" : [ "obj-174", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-184", 0 ],
									"midpoints" : [ 282.0, 403.0, 51.5, 403.0, 51.5, 364.0, 75.0, 364.0 ],
									"order" : 1,
									"source" : [ "obj-174", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-233", 0 ],
									"source" : [ "obj-184", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-244", 0 ],
									"source" : [ "obj-184", 5 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-247", 0 ],
									"source" : [ "obj-184", 4 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-251", 0 ],
									"source" : [ "obj-184", 3 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-257", 0 ],
									"source" : [ "obj-184", 2 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-258", 0 ],
									"source" : [ "obj-184", 1 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-168", 0 ],
									"midpoints" : [ 222.0, 388.0, 147.5, 388.0, 147.5, 331.0, 75.0, 331.0 ],
									"source" : [ "obj-185", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-172", 1 ],
									"midpoints" : [ 244.0, 526.0, 343.5, 526.0 ],
									"source" : [ "obj-185", 2 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-174", 0 ],
									"midpoints" : [ 255.0, 388.0, 267.5, 388.0, 267.5, 361.0, 282.0, 361.0 ],
									"source" : [ "obj-185", 3 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-201", 0 ],
									"midpoints" : [ 233.0, 388.0, 171.5, 388.0, 171.5, 193.0, 135.5, 193.0 ],
									"source" : [ "obj-185", 1 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-185", 0 ],
									"midpoints" : [ 222.0, 361.206395149230957, 222.0, 361.206395149230957 ],
									"source" : [ "obj-186", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-197", 1 ],
									"midpoints" : [ 204.0, 363.432215571403503, 214.5, 363.432215571403503 ],
									"source" : [ "obj-187", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-198", 1 ],
									"midpoints" : [ 74.0, 232.0, 85.5, 232.0 ],
									"source" : [ "obj-188", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-196", 1 ],
									"midpoints" : [ 204.0, 286.0, 152.0, 286.0 ],
									"source" : [ "obj-190", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-196", 0 ],
									"midpoints" : [ 75.0, 286.0, 124.0, 286.0 ],
									"source" : [ "obj-191", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-197", 0 ],
									"midpoints" : [ 177.0, 232.0, 204.0, 232.0 ],
									"source" : [ "obj-192", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-198", 0 ],
									"midpoints" : [ 74.0, 190.0, 51.5, 190.0, 51.5, 232.0, 75.0, 232.0 ],
									"source" : [ "obj-194", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-184", 1 ],
									"midpoints" : [ 124.0, 364.0, 127.5, 364.0 ],
									"source" : [ "obj-196", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-190", 0 ],
									"midpoints" : [ 204.0, 393.432215571403503, 204.0, 393.432215571403503 ],
									"source" : [ "obj-197", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-191", 0 ],
									"midpoints" : [ 75.0, 259.0, 75.0, 259.0 ],
									"source" : [ "obj-198", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-194", 0 ],
									"order" : 1,
									"source" : [ "obj-2", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-201", 0 ],
									"order" : 0,
									"source" : [ "obj-2", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-197", 0 ],
									"midpoints" : [ 135.5, 235.0, 189.5, 235.0, 189.5, 232.0, 204.0, 232.0 ],
									"order" : 0,
									"source" : [ "obj-201", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-198", 0 ],
									"midpoints" : [ 135.5, 235.0, 96.5, 235.0, 96.5, 232.0, 75.0, 232.0 ],
									"order" : 1,
									"source" : [ "obj-201", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-187", 0 ],
									"midpoints" : [ 174.0, 139.0, 162.5, 139.0, 162.5, 196.0, 198.5, 196.0, 198.5, 202.0, 204.0, 202.0 ],
									"source" : [ "obj-202", 3 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-188", 0 ],
									"midpoints" : [ 155.666666666666657, 133.0, 96.5, 133.0, 96.5, 127.0, 51.5, 127.0, 51.5, 202.0, 74.0, 202.0 ],
									"source" : [ "obj-202", 2 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-190", 1 ],
									"midpoints" : [ 137.333333333333343, 151.0, 162.5, 151.0, 162.5, 196.0, 171.5, 196.0, 171.5, 259.0, 214.5, 259.0 ],
									"source" : [ "obj-202", 1 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-191", 1 ],
									"midpoints" : [ 119.0, 124.0, 51.5, 124.0, 51.5, 259.0, 85.5, 259.0 ],
									"source" : [ "obj-202", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-186", 0 ],
									"source" : [ "obj-226", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-168", 0 ],
									"order" : 1,
									"source" : [ "obj-227", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-174", 2 ],
									"order" : 0,
									"source" : [ "obj-227", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-202", 0 ],
									"source" : [ "obj-228", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-192", 0 ],
									"order" : 0,
									"source" : [ "obj-3", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-201", 0 ],
									"order" : 1,
									"source" : [ "obj-3", 0 ]
								}

							}
 ]
					}
,
					"patching_rect" : [ 399.999996185302734, 1557.142842292785645, 134.5, 22.0 ],
					"text" : "p flagSystem"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-246",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 1247.619035720825195, 1571.428556442260742, 38.0, 22.0 ],
					"text" : "s add"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-247",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 438.095233917236328, 1533.333318710327148, 42.0, 22.0 ],
					"text" : "r clear"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-223",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 1480.952366828918457, 1676.190460205078125, 44.0, 22.0 ],
					"text" : "s clear"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-208",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 104.761903762817383, 1585.71427059173584, 30.0, 22.0 ],
					"text" : "s xy"
				}

			}
, 			{
				"box" : 				{
					"fontname" : "Arial",
					"fontsize" : 12.0,
					"id" : "obj-209",
					"linecount" : 3,
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 3,
					"outlettype" : [ "", "", "" ],
					"patching_rect" : [ 104.761903762817383, 1533.333318710327148, 126.288652718067169, 49.0 ],
					"text" : "getattr presentation_rect @listen 1",
					"varname" : "u712000177"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-206",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 1028.571418762207031, 1619.047603607177734, 24.0, 22.0 ],
					"text" : "s y"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-207",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 999.999990463256836, 1619.047603607177734, 24.0, 22.0 ],
					"text" : "s x"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-189",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 399.999996185302734, 1533.333318710327148, 28.0, 22.0 ],
					"text" : "r xy"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-142",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 633.333327293395996, 1795.238078117370605, 85.0, 22.0 ],
					"text" : "1. 0.98 0.31 0."
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-147",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 528.571423530578613, 1795.238078117370605, 99.0, 22.0 ],
					"text" : "1. 0.98 0.31 0.63"
				}

			}
, 			{
				"box" : 				{
					"attr" : "bgfillcolor",
					"id" : "obj-148",
					"maxclass" : "attrui",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 528.571423530578613, 1823.809506416320801, 150.0, 22.0 ]
				}

			}
, 			{
				"box" : 				{
					"attr" : "presentation_rect",
					"id" : "obj-149",
					"maxclass" : "attrui",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 528.571423530578613, 1761.904745101928711, 150.0, 22.0 ]
				}

			}
, 			{
				"box" : 				{
					"angle" : 270.0,
					"bgcolor" : [ 1.0, 0.98, 0.31, 0.63 ],
					"id" : "obj-150",
					"maxclass" : "panel",
					"mode" : 0,
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 528.571423530578613, 1857.142839431762695, 16.0, 16.0 ],
					"proportion" : 0.5,
					"shape" : 1
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-151",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 409.523805618286133, 1795.238078117370605, 92.0, 22.0 ],
					"text" : "1. 0.31 0.757 0."
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-152",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 299.999997138977051, 1795.238078117370605, 105.0, 22.0 ],
					"text" : "1. 0.31 0.757 0.63"
				}

			}
, 			{
				"box" : 				{
					"attr" : "bgfillcolor",
					"id" : "obj-153",
					"maxclass" : "attrui",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 299.999997138977051, 1828.5714111328125, 150.0, 22.0 ]
				}

			}
, 			{
				"box" : 				{
					"attr" : "presentation_rect",
					"id" : "obj-248",
					"maxclass" : "attrui",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 299.999997138977051, 1766.66664981842041, 150.0, 22.0 ]
				}

			}
, 			{
				"box" : 				{
					"angle" : 270.0,
					"bgcolor" : [ 1.0, 0.31, 0.757, 0.63 ],
					"id" : "obj-155",
					"maxclass" : "panel",
					"mode" : 0,
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 299.999997138977051, 1861.904744148254395, 16.0, 16.0 ],
					"proportion" : 0.5,
					"shape" : 1
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-156",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 199.999998092651367, 1799.999982833862305, 92.0, 22.0 ],
					"text" : "1. 0.435 0.31 0."
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-157",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 90.476189613342285, 1799.999982833862305, 105.0, 22.0 ],
					"text" : "1. 0.435 0.31 0.63"
				}

			}
, 			{
				"box" : 				{
					"attr" : "bgfillcolor",
					"id" : "obj-158",
					"maxclass" : "attrui",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 90.476189613342285, 1823.809506416320801, 150.0, 22.0 ]
				}

			}
, 			{
				"box" : 				{
					"attr" : "presentation_rect",
					"id" : "obj-161",
					"maxclass" : "attrui",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 90.476189613342285, 1766.66664981842041, 150.0, 22.0 ]
				}

			}
, 			{
				"box" : 				{
					"angle" : 270.0,
					"bgcolor" : [ 1.0, 0.435, 0.31, 0.63 ],
					"id" : "obj-162",
					"maxclass" : "panel",
					"mode" : 0,
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 90.476189613342285, 1857.142839431762695, 16.0, 16.0 ],
					"proportion" : 0.5,
					"shape" : 1
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-163",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 623.809517860412598, 1661.904746055603027, 85.0, 22.0 ],
					"text" : "1. 0.31 0.51 0."
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-164",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 514.285709381103516, 1661.904746055603027, 99.0, 22.0 ],
					"text" : "1. 0.31 0.51 0.63"
				}

			}
, 			{
				"box" : 				{
					"attr" : "bgfillcolor",
					"id" : "obj-165",
					"maxclass" : "attrui",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 514.285709381103516, 1690.476174354553223, 150.0, 22.0 ]
				}

			}
, 			{
				"box" : 				{
					"attr" : "presentation_rect",
					"id" : "obj-166",
					"maxclass" : "attrui",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 514.285709381103516, 1633.333317756652832, 150.0, 22.0 ]
				}

			}
, 			{
				"box" : 				{
					"angle" : 270.0,
					"bgcolor" : [ 1.0, 0.31, 0.51, 0.63 ],
					"id" : "obj-167",
					"maxclass" : "panel",
					"mode" : 0,
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 514.285709381103516, 1719.047602653503418, 16.0, 16.0 ],
					"proportion" : 0.5,
					"shape" : 1
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-175",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 414.285710334777832, 1661.904746055603027, 92.0, 22.0 ],
					"text" : "0.451 1. 0.31 0."
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-176",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 304.76190185546875, 1661.904746055603027, 105.0, 22.0 ],
					"text" : "0.451 1. 0.31 0.63"
				}

			}
, 			{
				"box" : 				{
					"attr" : "bgfillcolor",
					"id" : "obj-177",
					"maxclass" : "attrui",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 304.76190185546875, 1690.476174354553223, 150.0, 22.0 ]
				}

			}
, 			{
				"box" : 				{
					"attr" : "presentation_rect",
					"id" : "obj-178",
					"maxclass" : "attrui",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 304.76190185546875, 1633.333317756652832, 150.0, 22.0 ]
				}

			}
, 			{
				"box" : 				{
					"angle" : 270.0,
					"bgcolor" : [ 0.451, 1.0, 0.31, 0.63 ],
					"id" : "obj-179",
					"maxclass" : "panel",
					"mode" : 0,
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 333.333330154418945, 1719.047602653503418, 16.0, 16.0 ],
					"proportion" : 0.5,
					"shape" : 1
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-180",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 371.428567886352539, 1533.333318710327148, 24.0, 24.0 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_enum" : [ "off", "on" ],
							"parameter_longname" : "button[5]",
							"parameter_mmax" : 1,
							"parameter_modmode" : 0,
							"parameter_shortname" : "button[5]",
							"parameter_type" : 2
						}

					}
,
					"varname" : "button[5]"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-181",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 204.761902809143066, 1661.904746055603027, 92.0, 22.0 ],
					"text" : "0.31 0.588 1. 0."
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-182",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 99.999999046325684, 1661.904746055603027, 105.0, 22.0 ],
					"text" : "0.31 0.588 1. 0.63"
				}

			}
, 			{
				"box" : 				{
					"attr" : "bgfillcolor",
					"id" : "obj-183",
					"maxclass" : "attrui",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 99.999999046325684, 1690.476174354553223, 150.0, 22.0 ]
				}

			}
, 			{
				"box" : 				{
					"attr" : "presentation_rect",
					"id" : "obj-199",
					"maxclass" : "attrui",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 99.999999046325684, 1633.333317756652832, 150.0, 22.0 ]
				}

			}
, 			{
				"box" : 				{
					"angle" : 270.0,
					"bgcolor" : [ 0.31, 0.588, 1.0, 0.63 ],
					"id" : "obj-200",
					"maxclass" : "panel",
					"mode" : 0,
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 99.999999046325684, 1719.047602653503418, 16.0, 16.0 ],
					"proportion" : 0.5,
					"shape" : 1
				}

			}
, 			{
				"box" : 				{
					"bgcolor" : [ 0.0, 0.188235294117647, 0.286274509803922, 1.0 ],
					"bgcolor2" : [ 0.2, 0.2, 0.2, 1.0 ],
					"bgfillcolor_angle" : 270.0,
					"bgfillcolor_autogradient" : 0.0,
					"bgfillcolor_color" : [ 0.0, 0.188235294117647, 0.286274509803922, 1.0 ],
					"bgfillcolor_color1" : [ 0.0, 0.188235294117647, 0.286274509803922, 1.0 ],
					"bgfillcolor_color2" : [ 0.2, 0.2, 0.2, 1.0 ],
					"bgfillcolor_proportion" : 0.5,
					"bgfillcolor_type" : "gradient",
					"gradient" : 1,
					"id" : "obj-146",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1066.666656494140625, 1871.428553581237793, 33.0, 22.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 1479.999858856201172, 530.71610414981842, 33.0, 22.0 ],
					"text" : "read",
					"textcolor" : [ 0.992156862745098, 0.941176470588235, 0.835294117647059, 1.0 ]
				}

			}
, 			{
				"box" : 				{
					"bgcolor" : [ 0.0, 0.188235294117647, 0.286274509803922, 1.0 ],
					"bgcolor2" : [ 0.2, 0.2, 0.2, 1.0 ],
					"bgfillcolor_angle" : 270.0,
					"bgfillcolor_autogradient" : 0.0,
					"bgfillcolor_color" : [ 0.0, 0.188235294117647, 0.286274509803922, 1.0 ],
					"bgfillcolor_color1" : [ 0.0, 0.188235294117647, 0.286274509803922, 1.0 ],
					"bgfillcolor_color2" : [ 0.2, 0.2, 0.2, 1.0 ],
					"bgfillcolor_proportion" : 0.5,
					"bgfillcolor_type" : "gradient",
					"gradient" : 1,
					"id" : "obj-144",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1061.904751777648926, 1842.857125282287598, 34.0, 22.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 1439.999862670898438, 530.71610414981842, 34.0, 22.0 ],
					"text" : "write",
					"textcolor" : [ 0.992156862745098, 0.941176470588235, 0.835294117647059, 1.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-250",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 757.142849922180176, 1980.952362060546875, 89.0, 22.0 ],
					"text" : "s paramsbuflist"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-118",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 804.761897087097168, 1780.952363967895508, 35.0, 22.0 ],
					"text" : "clear"
				}

			}
, 			{
				"box" : 				{
					"bubble" : 1,
					"id" : "obj-251",
					"linecount" : 4,
					"maxclass" : "comment",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 995.238085746765137, 1566.666651725769043, 117.0, 64.0 ],
					"text" : "scroll across the 2D plane to morph between synth presets"
				}

			}
, 			{
				"box" : 				{
					"bgcolor" : [ 1.0, 0.788235, 0.470588, 1.0 ],
					"fontname" : "Arial Bold",
					"hint" : "",
					"id" : "obj-120",
					"ignoreclick" : 1,
					"legacytextcolor" : 1,
					"maxclass" : "textbutton",
					"numinlets" : 1,
					"numoutlets" : 3,
					"outlettype" : [ "", "", "int" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 1109.523798942565918, 1585.71427059173584, 20.0, 20.0 ],
					"rounded" : 60.0,
					"text" : "7",
					"textcolor" : [ 0.34902, 0.34902, 0.34902, 1.0 ]
				}

			}
, 			{
				"box" : 				{
					"bgcolor" : [ 1.0, 0.788235, 0.470588, 1.0 ],
					"fontname" : "Arial Bold",
					"hint" : "",
					"id" : "obj-121",
					"ignoreclick" : 1,
					"legacytextcolor" : 1,
					"maxclass" : "textbutton",
					"numinlets" : 1,
					"numoutlets" : 3,
					"outlettype" : [ "", "", "int" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 866.666658401489258, 1628.571413040161133, 20.0, 20.0 ],
					"rounded" : 60.0,
					"text" : "6",
					"textcolor" : [ 0.34902, 0.34902, 0.34902, 1.0 ]
				}

			}
, 			{
				"box" : 				{
					"bubble" : 1,
					"bubbleside" : 2,
					"id" : "obj-122",
					"linecount" : 2,
					"maxclass" : "comment",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 761.904754638671875, 1614.285698890686035, 103.786815941333771, 52.0 ],
					"text" : "switch to predicting points"
				}

			}
, 			{
				"box" : 				{
					"bubble" : 1,
					"bubbleside" : 2,
					"id" : "obj-123",
					"linecount" : 3,
					"maxclass" : "comment",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 933.333324432373047, 1742.857126235961914, 159.0, 66.0 ],
					"text" : "click to teach mlpregressor, repeat to get as close to zero error as possible"
				}

			}
, 			{
				"box" : 				{
					"bgcolor" : [ 1.0, 0.788235, 0.470588, 1.0 ],
					"fontname" : "Arial Bold",
					"hint" : "",
					"id" : "obj-252",
					"ignoreclick" : 1,
					"legacytextcolor" : 1,
					"maxclass" : "textbutton",
					"numinlets" : 1,
					"numoutlets" : 3,
					"outlettype" : [ "", "", "int" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 1090.476180076599121, 1766.66664981842041, 20.0, 20.0 ],
					"rounded" : 60.0,
					"text" : "5",
					"textcolor" : [ 0.34902, 0.34902, 0.34902, 1.0 ]
				}

			}
, 			{
				"box" : 				{
					"bubble" : 1,
					"id" : "obj-124",
					"linecount" : 3,
					"maxclass" : "comment",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 1299.999987602233887, 1523.80950927734375, 195.157894134521484, 51.0 ],
					"text" : "click to add the settings to the datasets. repeat steps 2 to 4 to add more data to the datasets"
				}

			}
, 			{
				"box" : 				{
					"bubble" : 1,
					"id" : "obj-125",
					"linecount" : 2,
					"maxclass" : "comment",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 995.238085746765137, 1528.571413993835449, 111.157894134521484, 37.0 ],
					"text" : "choose a point of the 2D plane"
				}

			}
, 			{
				"box" : 				{
					"bgcolor" : [ 1.0, 0.788235, 0.470588, 1.0 ],
					"fontname" : "Arial Bold",
					"hint" : "",
					"id" : "obj-36",
					"ignoreclick" : 1,
					"legacytextcolor" : 1,
					"maxclass" : "textbutton",
					"numinlets" : 1,
					"numoutlets" : 3,
					"outlettype" : [ "", "", "int" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 1109.523798942565918, 1533.333318710327148, 20.0, 20.0 ],
					"rounded" : 60.0,
					"text" : "2",
					"textcolor" : [ 0.34902, 0.34902, 0.34902, 1.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-126",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 904.761896133422852, 1709.52379322052002, 24.0, 24.0 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_enum" : [ "off", "on" ],
							"parameter_longname" : "button[4]",
							"parameter_mmax" : 1,
							"parameter_modmode" : 0,
							"parameter_shortname" : "button[4]",
							"parameter_type" : 2
						}

					}
,
					"varname" : "button[4]"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-127",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 761.904754638671875, 1671.428555488586426, 70.0, 22.0 ],
					"text" : "loadmess 0"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 13.0,
					"id" : "obj-128",
					"maxclass" : "comment",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 776.190468788146973, 1704.76188850402832, 109.0, 21.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 1079.999897003173828, 703.216087698936462, 109.0, 21.0 ],
					"text" : "selecting points",
					"textcolor" : [ 0.992156862745098, 0.941176470588235, 0.835294117647059, 1.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-253",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 761.904754638671875, 1752.380935668945312, 163.0, 22.0 ],
					"text" : "gate"
				}

			}
, 			{
				"box" : 				{
					"activecolor" : [ 0.992156862745098, 0.941176470588235, 0.835294117647059, 1.0 ],
					"bgcolor" : [ 0.0, 0.188235294117647, 0.286274509803922, 1.0 ],
					"disabled" : [ 0, 0 ],
					"elementcolor" : [ 0.0, 0.188235294117647, 0.286274509803922, 1.0 ],
					"id" : "obj-8",
					"itemtype" : 0,
					"maxclass" : "radiogroup",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 761.904754638671875, 1704.76188850402832, 123.0, 34.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 1064.999898433685303, 705.716087460517883, 123.0, 34.0 ],
					"size" : 2,
					"value" : 0
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-42",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "list" ],
					"patching_rect" : [ 757.142849922180176, 1952.38093376159668, 183.0, 22.0 ],
					"text" : "fluid.buf2list @source paramsbuf"
				}

			}
, 			{
				"box" : 				{
					"bgcolor" : [ 0.0, 0.188235294117647, 0.286274509803922, 1.0 ],
					"bgcolor2" : [ 0.2, 0.2, 0.2, 1.0 ],
					"bgfillcolor_angle" : 270.0,
					"bgfillcolor_autogradient" : 0.0,
					"bgfillcolor_color" : [ 0.0, 0.188235294117647, 0.286274509803922, 1.0 ],
					"bgfillcolor_color1" : [ 0.0, 0.188235294117647, 0.286274509803922, 1.0 ],
					"bgfillcolor_color2" : [ 0.2, 0.2, 0.2, 1.0 ],
					"bgfillcolor_proportion" : 0.5,
					"bgfillcolor_type" : "gradient",
					"gradient" : 1,
					"id" : "obj-41",
					"ignoreclick" : 1,
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 757.142849922180176, 1904.761886596679688, 131.0, 22.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 1302.499875783920288, 530.71610414981842, 131.0, 22.0 ],
					"text" : "clear",
					"textcolor" : [ 0.992156862745098, 0.941176470588235, 0.835294117647059, 1.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-254",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 2,
					"outlettype" : [ "bang", "" ],
					"patching_rect" : [ 757.142849922180176, 1928.571410179138184, 89.0, 22.0 ],
					"text" : "sel predictpoint"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-38",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 761.904754638671875, 1809.523792266845703, 163.0, 22.0 ],
					"text" : "predictpoint xybuf paramsbuf"
				}

			}
, 			{
				"box" : 				{
					"bgcolor" : [ 0.0, 0.188235294117647, 0.286274509803922, 1.0 ],
					"bgcolor2" : [ 0.2, 0.2, 0.2, 1.0 ],
					"bgfillcolor_angle" : 270.0,
					"bgfillcolor_autogradient" : 0.0,
					"bgfillcolor_color" : [ 0.0, 0.188235294117647, 0.286274509803922, 1.0 ],
					"bgfillcolor_color1" : [ 0.0, 0.188235294117647, 0.286274509803922, 1.0 ],
					"bgfillcolor_color2" : [ 0.2, 0.2, 0.2, 1.0 ],
					"bgfillcolor_proportion" : 0.5,
					"bgfillcolor_type" : "gradient",
					"gradient" : 1,
					"id" : "obj-129",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 933.333324432373047, 1809.523792266845703, 123.0, 22.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 1302.499875783920288, 435.716113209724426, 123.0, 22.0 ],
					"text" : "fit xydata paramsdata",
					"textcolor" : [ 0.992156862745098, 0.941176470588235, 0.835294117647059, 1.0 ]
				}

			}
, 			{
				"box" : 				{
					"bgcolor" : [ 0.0, 0.188235294117647, 0.286274509803922, 1.0 ],
					"bgcolor2" : [ 0.2, 0.2, 0.2, 1.0 ],
					"bgfillcolor_angle" : 270.0,
					"bgfillcolor_autogradient" : 0.0,
					"bgfillcolor_color" : [ 0.0, 0.188235294117647, 0.286274509803922, 1.0 ],
					"bgfillcolor_color1" : [ 0.0, 0.188235294117647, 0.286274509803922, 1.0 ],
					"bgfillcolor_color2" : [ 0.2, 0.2, 0.2, 1.0 ],
					"bgfillcolor_proportion" : 0.5,
					"bgfillcolor_type" : "gradient",
					"gradient" : 1,
					"id" : "obj-130",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1480.952366828918457, 1642.85712718963623, 35.0, 22.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 937.499910593032837, 710.716086983680725, 35.0, 22.0 ],
					"text" : "clear",
					"textcolor" : [ 0.992156862745098, 0.941176470588235, 0.835294117647059, 1.0 ]
				}

			}
, 			{
				"box" : 				{
					"bgcolor" : [ 0.0, 0.188235294117647, 0.286274509803922, 1.0 ],
					"bgcolor2" : [ 0.2, 0.2, 0.2, 1.0 ],
					"bgfillcolor_angle" : 270.0,
					"bgfillcolor_autogradient" : 0.0,
					"bgfillcolor_color" : [ 0.0, 0.188235294117647, 0.286274509803922, 1.0 ],
					"bgfillcolor_color1" : [ 0.0, 0.188235294117647, 0.286274509803922, 1.0 ],
					"bgfillcolor_color2" : [ 0.2, 0.2, 0.2, 1.0 ],
					"bgfillcolor_proportion" : 0.5,
					"bgfillcolor_type" : "gradient",
					"gradient" : 1,
					"id" : "obj-131",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1438.095224380493164, 1642.85712718963623, 32.0, 22.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 982.499906301498413, 710.716086983680725, 32.0, 22.0 ],
					"text" : "print",
					"textcolor" : [ 0.992156862745098, 0.941176470588235, 0.835294117647059, 1.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-132",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1180.952369689941406, 1642.85712718963623, 103.0, 22.0 ],
					"text" : "addpoint $1 xybuf"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-133",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1304.761892318725586, 1642.85712718963623, 131.0, 22.0 ],
					"text" : "addpoint $1 paramsbuf"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-23",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "" ],
					"patching_rect" : [ 1304.761892318725586, 1676.190460205078125, 147.0, 22.0 ],
					"text" : "fluid.dataset~ paramsdata"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-22",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "" ],
					"patching_rect" : [ 1180.952369689941406, 1676.190460205078125, 119.0, 22.0 ],
					"text" : "fluid.dataset~ xydata"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-134",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 2,
					"outlettype" : [ "", "" ],
					"patching_rect" : [ 1180.952369689941406, 1609.523794174194336, 162.0, 22.0 ],
					"text" : "combine point- s @triggers 1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-20",
					"maxclass" : "newobj",
					"numinlets" : 5,
					"numoutlets" : 4,
					"outlettype" : [ "int", "", "", "int" ],
					"patching_rect" : [ 1180.952369689941406, 1571.428556442260742, 61.0, 22.0 ],
					"text" : "counter"
				}

			}
, 			{
				"box" : 				{
					"bgcolor" : [ 0.0, 0.188235294117647, 0.286274509803922, 1.0 ],
					"blinkcolor" : [ 0.992156862745098, 0.941176470588235, 0.835294117647059, 1.0 ],
					"id" : "obj-13",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"outlinecolor" : [ 0.992156862745098, 0.941176470588235, 0.835294117647059, 1.0 ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1180.952369689941406, 1538.095223426818848, 24.0, 24.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 862.49991774559021, 710.716086983680725, 22.354165375232697, 22.354165375232697 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_enum" : [ "off", "on" ],
							"parameter_longname" : "button[3]",
							"parameter_mmax" : 1,
							"parameter_modmode" : 0,
							"parameter_shortname" : "button[3]",
							"parameter_type" : 2
						}

					}
,
					"varname" : "button[3]"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-137",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "buffer" ],
					"patching_rect" : [ 1180.952369689941406, 1847.619029998779297, 205.0, 22.0 ],
					"text" : "fluid.list2buf @destination paramsbuf"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-255",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 904.761896133422852, 1652.380936622619629, 91.0, 22.0 ],
					"text" : "vexpr $i1 / 127."
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-256",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 904.761896133422852, 1619.047603607177734, 86.0, 22.0 ],
					"text" : "join"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-138",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "float", "bang" ],
					"patching_rect" : [ 942.857133865356445, 1709.52379322052002, 139.0, 22.0 ],
					"text" : "buffer~ xybuf @samps 2"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-3",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "buffer" ],
					"patching_rect" : [ 904.761896133422852, 1680.952364921569824, 177.0, 22.0 ],
					"text" : "fluid.list2buf @destination xybuf"
				}

			}
, 			{
				"box" : 				{
					"bgcolor" : [ 0.0, 0.188235294117647, 0.286274509803922, 1.0 ],
					"color" : [ 1.0, 0.756862745098039, 0.03921568627451, 1.0 ],
					"id" : "obj-257",
					"linecount" : 3,
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "" ],
					"patching_rect" : [ 761.904754638671875, 1842.857125282287598, 294.0, 49.0 ],
					"presentation" : 1,
					"presentation_linecount" : 4,
					"presentation_rect" : [ 1302.499875783920288, 460.716110825538635, 214.0, 62.0 ],
					"text" : "fluid.mlpregressor~ @hiddenlayers 10 @activation 1 @outputactivation 1 @batchsize 1 @maxiter 10000 @learnrate 0.1 @validation 0",
					"textcolor" : [ 0.992156862745098, 0.941176470588235, 0.835294117647059, 1.0 ]
				}

			}
, 			{
				"box" : 				{
					"fontname" : "Lato",
					"fontsize" : 18.0,
					"id" : "obj-24",
					"maxclass" : "comment",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 761.904754638671875, 1528.571413993835449, 100.0, 28.0 ],
					"text" : "Regression"
				}

			}
, 			{
				"box" : 				{
					"bgcolor" : [ 0.0, 0.188235294117647, 0.286274509803922, 1.0 ],
					"color" : [ 0.992156862745098, 0.941176470588235, 0.835294117647059, 1.0 ],
					"id" : "obj-205",
					"maxclass" : "pictslider",
					"numinlets" : 2,
					"numoutlets" : 2,
					"outlettype" : [ "int", "int" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 904.761896133422852, 1528.571413993835449, 86.0, 83.0 ],
					"style" : "rnbohighcontrast"
				}

			}
, 			{
				"box" : 				{
					"angle" : 270.0,
					"bgcolor" : [ 0.992156862745098, 0.941176470588235, 0.835294117647059, 1.0 ],
					"id" : "obj-211",
					"maxclass" : "panel",
					"mode" : 0,
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 76.190475463867188, 1509.523795127868652, 662.0, 382.0 ],
					"proportion" : 0.5,
					"rounded" : 19
				}

			}
, 			{
				"box" : 				{
					"angle" : 270.0,
					"bgcolor" : [ 0.992156862745098, 0.941176470588235, 0.835294117647059, 1.0 ],
					"id" : "obj-258",
					"maxclass" : "panel",
					"mode" : 0,
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 747.619040489196777, 1509.523795127868652, 391.0, 503.0 ],
					"proportion" : 0.5,
					"rounded" : 19
				}

			}
, 			{
				"box" : 				{
					"angle" : 270.0,
					"bgcolor" : [ 0.992156862745098, 0.941176470588235, 0.835294117647059, 1.0 ],
					"id" : "obj-220",
					"maxclass" : "panel",
					"mode" : 0,
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 1152.380941390991211, 1509.523795127868652, 426.0, 385.0 ],
					"proportion" : 0.5,
					"rounded" : 19
				}

			}
, 			{
				"box" : 				{
					"automatic" : 1,
					"bufsize" : 48,
					"calccount" : 62,
					"id" : "obj-241",
					"maxclass" : "scope~",
					"numinlets" : 2,
					"numoutlets" : 0,
					"patching_rect" : [ 357.934808790683746, 447.777799129486084, 128.888895034790039, 76.666670322418213 ],
					"presentation" : 1,
					"presentation_rect" : [ 406.666678786277771, 19.333333909511566, 130.0, 130.0 ]
				}

			}
, 			{
				"box" : 				{
					"candicane2" : [ 0.65098, 0.666667, 0.662745, 1.0 ],
					"candycane" : 2,
					"contdata" : 1,
					"id" : "obj-9",
					"maxclass" : "multislider",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 78.125, 26.530611991882324, 220.0, 147.0 ],
					"presentation" : 1,
					"presentation_rect" : [ -278.102188289165497, 48.905109226703644, 331.0, 155.0 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_invisible" : 1,
							"parameter_longname" : "multislider",
							"parameter_modmode" : 0,
							"parameter_shortname" : "multislider",
							"parameter_type" : 3
						}

					}
,
					"setminmax" : [ 0.0, 1.0 ],
					"setstyle" : 1,
					"shadowblend" : 0.3,
					"shadoworientation" : 3,
					"shadowperbar" : 0,
					"size" : 8,
					"slidercolor" : [ 0.862745, 0.870588, 0.878431, 1.0 ],
					"style" : "velvet",
					"varname" : "multislider"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 8.0,
					"id" : "obj-228",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 481.585136473178864, 154.41176176071167, 27.0, 35.0 ],
					"presentation" : 1,
					"presentation_rect" : [ -22.627737104892731, 206.569341957569122, 27.0, 35.0 ],
					"saved_attribute_attributes" : 					{
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_initial" : [ 8.0 ],
							"parameter_initial_enable" : 1,
							"parameter_longname" : "-[4]",
							"parameter_mmax" : 18.0,
							"parameter_mmin" : 1.0,
							"parameter_modmode" : 4,
							"parameter_shortname" : "1",
							"parameter_type" : 1,
							"parameter_unitstyle" : 0
						}

					}
,
					"showname" : 0,
					"textcolor" : [ 0.086275, 0.309804, 0.52549, 1.0 ],
					"varname" : "live.dial[6]"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 8.0,
					"id" : "obj-229",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 424.232196390628815, 154.41176176071167, 27.0, 35.0 ],
					"presentation" : 1,
					"presentation_rect" : [ -63.503649294376373, 206.569341957569122, 27.0, 35.0 ],
					"saved_attribute_attributes" : 					{
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_initial" : [ 6.0 ],
							"parameter_initial_enable" : 1,
							"parameter_longname" : "-[5]",
							"parameter_mmax" : 18.0,
							"parameter_mmin" : 1.0,
							"parameter_modmode" : 4,
							"parameter_shortname" : "1",
							"parameter_type" : 1,
							"parameter_unitstyle" : 0
						}

					}
,
					"showname" : 0,
					"textcolor" : [ 0.086275, 0.309804, 0.52549, 1.0 ],
					"varname" : "live.dial[7]"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 8.0,
					"id" : "obj-230",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 366.879256308078766, 154.41176176071167, 27.0, 35.0 ],
					"presentation" : 1,
					"presentation_rect" : [ -105.109488487243652, 206.569341957569122, 27.0, 35.0 ],
					"saved_attribute_attributes" : 					{
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_initial" : [ 5.0 ],
							"parameter_initial_enable" : 1,
							"parameter_longname" : "-[6]",
							"parameter_mmax" : 18.0,
							"parameter_mmin" : 1.0,
							"parameter_modmode" : 4,
							"parameter_shortname" : "1",
							"parameter_type" : 1,
							"parameter_unitstyle" : 0
						}

					}
,
					"showname" : 0,
					"textcolor" : [ 0.086275, 0.309804, 0.52549, 1.0 ],
					"varname" : "live.dial[8]"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 8.0,
					"id" : "obj-231",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 540.408664762973785, 72.058822154998779, 27.0, 35.0 ],
					"presentation" : 1,
					"presentation_rect" : [ -145.985400676727295, 206.569341957569122, 27.0, 35.0 ],
					"saved_attribute_attributes" : 					{
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "-[7]",
							"parameter_mmax" : 18.0,
							"parameter_mmin" : 1.0,
							"parameter_modmode" : 4,
							"parameter_shortname" : "1",
							"parameter_type" : 1,
							"parameter_unitstyle" : 0
						}

					}
,
					"showname" : 0,
					"textcolor" : [ 0.086275, 0.309804, 0.52549, 1.0 ],
					"varname" : "live.dial[9]"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-232",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 481.585136473178864, 197.058819770812988, 32.0, 22.0 ],
					"text" : "7 $1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-233",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 424.232196390628815, 197.058819770812988, 32.0, 22.0 ],
					"text" : "6 $1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-234",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 366.879256308078766, 197.058819770812988, 32.0, 22.0 ],
					"text" : "5 $1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-235",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 540.408664762973785, 117.647056579589844, 32.0, 22.0 ],
					"text" : "4 $1"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 8.0,
					"id" : "obj-227",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 547.408664762973785, 156.411050260066986, 27.0, 35.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 18.248175084590912, 206.569341957569122, 27.0, 35.0 ],
					"saved_attribute_attributes" : 					{
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_initial" : [ 10.0 ],
							"parameter_initial_enable" : 1,
							"parameter_longname" : "-[3]",
							"parameter_mmax" : 18.0,
							"parameter_mmin" : 1.0,
							"parameter_modmode" : 4,
							"parameter_shortname" : "1",
							"parameter_type" : 1,
							"parameter_unitstyle" : 0
						}

					}
,
					"showname" : 0,
					"textcolor" : [ 0.086275, 0.309804, 0.52549, 1.0 ],
					"varname" : "live.dial[5]"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 8.0,
					"id" : "obj-226",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 483.055724680423737, 72.058822154998779, 27.0, 35.0 ],
					"presentation" : 1,
					"presentation_rect" : [ -187.591239869594574, 206.569341957569122, 27.0, 35.0 ],
					"saved_attribute_attributes" : 					{
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "-[2]",
							"parameter_mmax" : 18.0,
							"parameter_mmin" : 1.0,
							"parameter_modmode" : 4,
							"parameter_shortname" : "1",
							"parameter_type" : 1,
							"parameter_unitstyle" : 0
						}

					}
,
					"showname" : 0,
					"textcolor" : [ 0.086275, 0.309804, 0.52549, 1.0 ],
					"varname" : "live.dial[4]"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 8.0,
					"id" : "obj-225",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 425.702784597873688, 72.058822154998779, 27.0, 35.0 ],
					"presentation" : 1,
					"presentation_rect" : [ -228.467152059078217, 206.569341957569122, 27.0, 35.0 ],
					"saved_attribute_attributes" : 					{
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "-[1]",
							"parameter_mmax" : 18.0,
							"parameter_mmin" : 1.0,
							"parameter_modmode" : 4,
							"parameter_shortname" : "1",
							"parameter_type" : 1,
							"parameter_unitstyle" : 0
						}

					}
,
					"showname" : 0,
					"textcolor" : [ 0.086275, 0.309804, 0.52549, 1.0 ],
					"varname" : "live.dial[3]"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 8.0,
					"id" : "obj-224",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 366.879256308078766, 72.058822154998779, 27.0, 35.0 ],
					"presentation" : 1,
					"presentation_rect" : [ -270.072991251945496, 205.839414954185486, 27.0, 35.0 ],
					"saved_attribute_attributes" : 					{
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "-",
							"parameter_mmax" : 18.0,
							"parameter_mmin" : 1.0,
							"parameter_modmode" : 4,
							"parameter_shortname" : "1",
							"parameter_type" : 1,
							"parameter_unitstyle" : 0
						}

					}
,
					"showname" : 0,
					"textcolor" : [ 0.086275, 0.309804, 0.52549, 1.0 ],
					"varname" : "live.dial"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-222",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 536.908664762973785, 217.977545499801636, 32.0, 22.0 ],
					"text" : "8 $1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-218",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 483.055724680423737, 117.647056579589844, 32.0, 22.0 ],
					"text" : "3 $1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-213",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 425.702784597873688, 117.647056579589844, 32.0, 22.0 ],
					"text" : "2 $1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-184",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 366.879256308078766, 117.647056579589844, 32.0, 22.0 ],
					"text" : "1 $1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-57",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "multichannelsignal" ],
					"patching_rect" : [ 320.0, 350.54946768283844, 53.0, 22.0 ],
					"text" : "mc.*~ 1."
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-46",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 3,
					"outlettype" : [ "multichannelsignal", "", "" ],
					"patching_rect" : [ 366.879256308078766, 288.235288619995117, 111.0, 22.0 ],
					"text" : "mc.line~ @chans 8"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-47",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 366.879256308078766, 255.88234806060791, 107.0, 22.0 ],
					"text" : "setvalue $1 $2 25."
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-168",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 127.472533702850342, 286.813200831413269, 24.0, 24.0 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_enum" : [ "off", "on" ],
							"parameter_longname" : "button",
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
					"id" : "obj-154",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 3,
					"outlettype" : [ "multichannelsignal", "", "" ],
					"patching_rect" : [ 84.615388751029968, 461.5384840965271, 111.0, 22.0 ],
					"text" : "mc.line~ @chans 8"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-139",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 84.615388751029968, 431.868152976036072, 107.0, 22.0 ],
					"text" : "setvalue $1 $2 25."
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-26",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 84.615388751029968, 403.296723008155823, 47.0, 22.0 ],
					"text" : "pack i f"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-19",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 84.615388751029968, 350.54946768283844, 47.0, 22.0 ],
					"text" : "pack f i"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-18",
					"maxclass" : "newobj",
					"numinlets" : 5,
					"numoutlets" : 4,
					"outlettype" : [ "int", "", "", "int" ],
					"patching_rect" : [ 104.395609498023987, 319.780235409736633, 69.0, 22.0 ],
					"text" : "counter 1 8"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-17",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "float", "bang" ],
					"patching_rect" : [ 84.615388751029968, 287.912101984024048, 29.5, 22.0 ],
					"text" : "t f b"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-14",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 2,
					"outlettype" : [ "", "" ],
					"patching_rect" : [ 84.615388751029968, 378.02199649810791, 45.0, 22.0 ],
					"text" : "zl.rot 1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-11",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 84.615388751029968, 262.068961143493652, 25.0, 22.0 ],
					"text" : "iter"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-7",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "multichannelsignal" ],
					"patching_rect" : [ 87.755101203918457, 580.219808578491211, 201.0, 22.0 ],
					"text" : "mc.mixdown~ @chans 1 @autogain"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-4",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 87.755101203918457, 489.115641593933105, 48.526148796081543, 48.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 113.0, 379.0, 47.674416899681091, 48.0 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_exponent" : 1.6,
							"parameter_initial" : [ 285.999999999999943 ],
							"parameter_initial_enable" : 1,
							"parameter_longname" : "live.dial",
							"parameter_mmax" : 400.0,
							"parameter_mmin" : 40.0,
							"parameter_modmode" : 4,
							"parameter_shortname" : "live.dial",
							"parameter_type" : 0,
							"parameter_unitstyle" : 3
						}

					}
,
					"varname" : "live.dial[10]"
				}

			}
, 			{
				"box" : 				{
					"data" : 					{
						"patcher" : 						{
							"fileversion" : 1,
							"appversion" : 							{
								"major" : 9,
								"minor" : 0,
								"revision" : 7,
								"architecture" : "x64",
								"modernui" : 1
							}
,
							"classnamespace" : "dsp.gen",
							"rect" : [ 34.0, 62.0, 1402.0, 860.0 ],
							"gridsize" : [ 15.0, 15.0 ],
							"boxes" : [ 								{
									"box" : 									{
										"maxclass" : "newobj",
										"text" : "*",
										"patching_rect" : [ 301.129999999999995, 291.379999999999995, 29.5, 22.0 ],
										"numinlets" : 2,
										"numoutlets" : 1,
										"outlettype" : [ "" ],
										"id" : "obj-3"
									}

								}
, 								{
									"box" : 									{
										"maxclass" : "newobj",
										"text" : "out 4",
										"patching_rect" : [ 313.160000000000025, 381.069999999999993, 35.0, 22.0 ],
										"numinlets" : 1,
										"numoutlets" : 0,
										"id" : "obj-1"
									}

								}
, 								{
									"box" : 									{
										"maxclass" : "newobj",
										"text" : "*",
										"patching_rect" : [ 168.0, 353.0, 29.5, 22.0 ],
										"numinlets" : 2,
										"numoutlets" : 1,
										"outlettype" : [ "" ],
										"id" : "obj-21"
									}

								}
, 								{
									"box" : 									{
										"maxclass" : "newobj",
										"text" : "out 3",
										"patching_rect" : [ 176.0, 389.0, 35.0, 22.0 ],
										"numinlets" : 1,
										"numoutlets" : 0,
										"id" : "obj-20"
									}

								}
, 								{
									"box" : 									{
										"maxclass" : "newobj",
										"text" : "scale -1. 1. 0.",
										"patching_rect" : [ 50.0, 319.0, 81.0, 22.0 ],
										"numinlets" : 3,
										"numoutlets" : 1,
										"outlettype" : [ "" ],
										"id" : "obj-12"
									}

								}
, 								{
									"box" : 									{
										"maxclass" : "newobj",
										"text" : "out 2",
										"patching_rect" : [ 167.0, 197.0, 35.0, 22.0 ],
										"numinlets" : 1,
										"numoutlets" : 0,
										"id" : "obj-14"
									}

								}
, 								{
									"box" : 									{
										"maxclass" : "newobj",
										"text" : "scale 0. 1. 0. 1. 1.3",
										"patching_rect" : [ 154.0, 165.0, 110.0, 22.0 ],
										"numinlets" : 1,
										"numoutlets" : 1,
										"outlettype" : [ "" ],
										"id" : "obj-13"
									}

								}
, 								{
									"box" : 									{
										"maxclass" : "newobj",
										"text" : "clip -1. 1.",
										"patching_rect" : [ 50.0, 354.0, 57.0, 22.0 ],
										"numinlets" : 1,
										"numoutlets" : 1,
										"outlettype" : [ "" ],
										"id" : "obj-4"
									}

								}
, 								{
									"box" : 									{
										"maxclass" : "newobj",
										"text" : "in 3",
										"patching_rect" : [ 562.0, 37.0, 28.0, 22.0 ],
										"numinlets" : 0,
										"numoutlets" : 1,
										"outlettype" : [ "" ],
										"id" : "obj-10"
									}

								}
, 								{
									"box" : 									{
										"maxclass" : "newobj",
										"text" : "in 2",
										"patching_rect" : [ 213.0, 128.0, 28.0, 22.0 ],
										"numinlets" : 0,
										"numoutlets" : 1,
										"outlettype" : [ "" ],
										"id" : "obj-8"
									}

								}
, 								{
									"box" : 									{
										"maxclass" : "newobj",
										"text" : "*",
										"patching_rect" : [ 50.0, 80.0, 29.5, 22.0 ],
										"numinlets" : 2,
										"numoutlets" : 1,
										"outlettype" : [ "" ],
										"id" : "obj-6"
									}

								}
, 								{
									"box" : 									{
										"maxclass" : "newobj",
										"text" : "cycle",
										"patching_rect" : [ 50.0, 115.0, 36.0, 22.0 ],
										"numinlets" : 1,
										"numoutlets" : 2,
										"outlettype" : [ "", "" ],
										"id" : "obj-5"
									}

								}
, 								{
									"box" : 									{
										"maxclass" : "newobj",
										"text" : "in 1",
										"patching_rect" : [ 50.0, 14.0, 28.0, 22.0 ],
										"numinlets" : 0,
										"numoutlets" : 1,
										"outlettype" : [ "" ],
										"id" : "obj-9"
									}

								}
, 								{
									"box" : 									{
										"maxclass" : "newobj",
										"text" : "out 1",
										"patching_rect" : [ 50.0, 389.0, 35.0, 22.0 ],
										"numinlets" : 1,
										"numoutlets" : 0,
										"id" : "obj-11"
									}

								}
 ],
							"lines" : [ 								{
									"patchline" : 									{
										"source" : [ "obj-4", 0 ],
										"destination" : [ "obj-11", 0 ]
									}

								}
, 								{
									"patchline" : 									{
										"source" : [ "obj-9", 0 ],
										"destination" : [ "obj-6", 0 ]
									}

								}
, 								{
									"patchline" : 									{
										"source" : [ "obj-6", 0 ],
										"destination" : [ "obj-5", 0 ]
									}

								}
, 								{
									"patchline" : 									{
										"source" : [ "obj-10", 0 ],
										"destination" : [ "obj-6", 1 ]
									}

								}
, 								{
									"patchline" : 									{
										"source" : [ "obj-8", 0 ],
										"destination" : [ "obj-13", 0 ],
										"order" : 1
									}

								}
, 								{
									"patchline" : 									{
										"source" : [ "obj-13", 0 ],
										"destination" : [ "obj-14", 0 ],
										"order" : 1
									}

								}
, 								{
									"patchline" : 									{
										"source" : [ "obj-5", 0 ],
										"destination" : [ "obj-12", 0 ],
										"order" : 2
									}

								}
, 								{
									"patchline" : 									{
										"source" : [ "obj-12", 0 ],
										"destination" : [ "obj-4", 0 ]
									}

								}
, 								{
									"patchline" : 									{
										"source" : [ "obj-13", 0 ],
										"destination" : [ "obj-12", 1 ],
										"order" : 2
									}

								}
, 								{
									"patchline" : 									{
										"source" : [ "obj-5", 0 ],
										"destination" : [ "obj-21", 0 ],
										"order" : 1
									}

								}
, 								{
									"patchline" : 									{
										"source" : [ "obj-21", 0 ],
										"destination" : [ "obj-20", 0 ]
									}

								}
, 								{
									"patchline" : 									{
										"source" : [ "obj-13", 0 ],
										"destination" : [ "obj-21", 1 ],
										"order" : 0
									}

								}
, 								{
									"patchline" : 									{
										"source" : [ "obj-5", 0 ],
										"destination" : [ "obj-3", 0 ],
										"order" : 0
									}

								}
, 								{
									"patchline" : 									{
										"source" : [ "obj-8", 0 ],
										"destination" : [ "obj-3", 1 ],
										"order" : 0
									}

								}
, 								{
									"patchline" : 									{
										"source" : [ "obj-3", 0 ],
										"destination" : [ "obj-1", 0 ]
									}

								}
 ]
						}

					}
,
					"id" : "obj-1",
					"maxclass" : "newobj",
					"numinlets" : 3,
					"numoutlets" : 4,
					"outlettype" : [ "multichannelsignal", "multichannelsignal", "multichannelsignal", "multichannelsignal" ],
					"patching_rect" : [ 87.755101203918457, 551.648378610610962, 112.0, 22.0 ],
					"text" : "mc.gen~ @chans 8",
					"wrapper_uniquekey" : "u153000882"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-62",
					"maxclass" : "live.menu",
					"numinlets" : 1,
					"numoutlets" : 3,
					"outlettype" : [ "", "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 71.625, 189.8125, 100.0, 15.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 479.338816404342651, 693.388391315937042, 100.0, 15.0 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_enum" : [ "Unscaled", "Scaled" ],
							"parameter_longname" : "live.menu",
							"parameter_mmax" : 1,
							"parameter_modmode" : 0,
							"parameter_shortname" : "live.menu",
							"parameter_type" : 2
						}

					}
,
					"varname" : "live.menu"
				}

			}
, 			{
				"box" : 				{
					"attr" : "active",
					"fontsize" : 8.0,
					"id" : "obj-80",
					"maxclass" : "attrui",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 1.472533702850342, 234.59375, 130.058997750282288, 17.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-221",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1099.99998950958252, 1247.619035720825195, 41.0, 22.0 ],
					"text" : "pause"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-236",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1142.857131958007812, 1247.619035720825195, 49.0, 22.0 ],
					"text" : "resume"
				}

			}
, 			{
				"box" : 				{
					"angle" : 270.0,
					"bgcolor" : [ 0.992156862745098, 0.941176470588235, 0.835294117647059, 1.0 ],
					"id" : "obj-272",
					"maxclass" : "panel",
					"mode" : 0,
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 923.809514999389648, 1238.095226287841797, 329.577469110488892, 253.521130084991455 ],
					"proportion" : 0.5,
					"rounded" : 19
				}

			}
, 			{
				"box" : 				{
					"angle" : 270.0,
					"bgcolor" : [ 0.184313725490196, 0.258823529411765, 0.172549019607843, 1.0 ],
					"bordercolor" : [ 0.952941176470588, 0.8, 0.619607843137255, 1.0 ],
					"id" : "obj-276",
					"maxclass" : "panel",
					"mode" : 0,
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 338.938080370426178, 16.176470279693604, 371.604967951774597, 302.469159960746765 ],
					"proportion" : 0.5,
					"rounded" : 19
				}

			}
, 			{
				"box" : 				{
					"angle" : 270.0,
					"bgcolor" : [ 0.184313725490196, 0.258823529411765, 0.172549019607843, 1.0 ],
					"bordercolor" : [ 0.952941176470588, 0.8, 0.619607843137255, 1.0 ],
					"id" : "obj-277",
					"maxclass" : "panel",
					"mode" : 0,
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 742.647044658660889, 16.176470279693604, 371.604967951774597, 302.469159960746765 ],
					"proportion" : 0.5,
					"rounded" : 19
				}

			}
, 			{
				"box" : 				{
					"background" : 1,
					"color" : [ 0.776470588235294, 0.627450980392157, 0.627450980392157, 0.301960784313725 ],
					"id" : "obj-5",
					"ignoreclick" : 1,
					"maxclass" : "mira.frame",
					"numinlets" : 0,
					"numoutlets" : 0,
					"patching_rect" : [ 233.889999999999986, 391.509999999999991, 256.000007629394531, 182.000000847503514 ],
					"presentation" : 1,
					"presentation_rect" : [ -284.671531319618225, 0.729927003383636, 682.101427734749336, 484.931471586227417 ]
				}

			}
 ],
		"lines" : [ 			{
				"patchline" : 				{
					"destination" : [ "obj-429", 0 ],
					"source" : [ "obj-1", 2 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-7", 0 ],
					"midpoints" : [ 97.255101203918457, 575.179628610610962, 97.255101203918457, 575.179628610610962 ],
					"source" : [ "obj-1", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-114", 0 ],
					"midpoints" : [ 891.852924346923828, 227.974127292633057, 777.14704418182373, 227.974127292633057 ],
					"source" : [ "obj-100", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-114", 0 ],
					"midpoints" : [ 833.029396057128906, 227.974127292633057, 777.14704418182373, 227.974127292633057 ],
					"source" : [ "obj-101", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-114", 0 ],
					"midpoints" : [ 777.14704418182373, 239.974127292633057, 777.14704418182373, 239.974127292633057 ],
					"source" : [ "obj-102", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-114", 0 ],
					"midpoints" : [ 950.67645263671875, 99.163955688476562, 936.128753423690796, 99.163955688476562, 936.128753423690796, 198.163955688476562, 777.14704418182373, 198.163955688476562 ],
					"source" : [ "obj-103", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-143", 0 ],
					"source" : [ "obj-104", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-117", 0 ],
					"midpoints" : [ 893.323512554168701, 69.163955688476562, 893.323512554168701, 69.163955688476562 ],
					"order" : 1,
					"source" : [ "obj-105", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-95", 2 ],
					"midpoints" : [ 893.323512554168701, 69.163955688476562, 990.128753423690796, 69.163955688476562, 990.128753423690796, 57.163955688476562, 1033.441157341003418, 57.163955688476562 ],
					"order" : 0,
					"source" : [ "obj-105", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-116", 0 ],
					"order" : 1,
					"source" : [ "obj-106", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-95", 1 ],
					"order" : 0,
					"source" : [ "obj-106", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-115", 0 ],
					"order" : 1,
					"source" : [ "obj-107", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-95", 0 ],
					"midpoints" : [ 777.14704418182373, 69.163955688476562, 810.128753423690796, 69.163955688476562, 810.128753423690796, 15.163955688476562, 1012.441157341003418, 15.163955688476562 ],
					"order" : 0,
					"source" : [ "obj-107", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-114", 0 ],
					"midpoints" : [ 950.67645263671875, 227.974127292633057, 777.14704418182373, 227.974127292633057 ],
					"source" : [ "obj-108", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-114", 0 ],
					"midpoints" : [ 891.852924346923828, 99.163955688476562, 879.128753423690796, 99.163955688476562, 879.128753423690796, 198.163955688476562, 777.14704418182373, 198.163955688476562 ],
					"source" : [ "obj-109", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-17", 0 ],
					"midpoints" : [ 94.115388751029968, 285.0, 94.115388751029968, 285.0 ],
					"source" : [ "obj-11", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-265", 3 ],
					"midpoints" : [ 519.023804664611816, 1555.083318710327148, 524.999996185302734, 1555.083318710327148 ],
					"source" : [ "obj-110", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-114", 0 ],
					"midpoints" : [ 834.499984264373779, 99.163955688476562, 819.128753423690796, 99.163955688476562, 819.128753423690796, 198.163955688476562, 777.14704418182373, 198.163955688476562 ],
					"source" : [ "obj-111", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-114", 0 ],
					"midpoints" : [ 775.676455974578857, 99.163955688476562, 753.128753423690796, 99.163955688476562, 753.128753423690796, 198.163955688476562, 777.14704418182373, 198.163955688476562 ],
					"source" : [ "obj-112", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-57", 0 ],
					"midpoints" : [ 774.205867767333984, 654.0, 402.0, 654.0, 402.0, 516.0, 329.5, 516.0 ],
					"source" : [ "obj-113", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-113", 0 ],
					"midpoints" : [ 777.14704418182373, 281.767771482467651, 774.205867767333984, 281.767771482467651 ],
					"source" : [ "obj-114", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-112", 0 ],
					"midpoints" : [ 777.14704418182373, 69.163955688476562, 775.676455974578857, 69.163955688476562 ],
					"source" : [ "obj-115", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-111", 0 ],
					"source" : [ "obj-116", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-109", 0 ],
					"source" : [ "obj-117", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-257", 0 ],
					"midpoints" : [ 814.261897087097168, 1804.324594855308533, 758.054966926574707, 1804.324594855308533, 758.054966926574707, 1834.324594855308533, 771.404754638671875, 1834.324594855308533 ],
					"source" : [ "obj-118", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-2", 7 ],
					"midpoints" : [ 297.735288619995117, 65.624999523162842, 409.427050650119781, 65.624999523162842, 409.427050650119781, 56.624999523162842, 526.427050650119781, 56.624999523162842, 526.427050650119781, 113.624999523162842, 535.427050650119781, 113.624999523162842, 535.427050650119781, 146.624999523162842, 549.908664762973785, 146.624999523162842, 549.908664762973785, 191.624999523162842, 577.427050650119781, 191.624999523162842, 577.427050650119781, 173.624999523162842, 694.427050650119781, 173.624999523162842, 694.427050650119781, 107.624999523162842, 685.173369467258453, 107.624999523162842 ],
					"order" : 3,
					"source" : [ "obj-12", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-224", 0 ],
					"midpoints" : [ 297.735288619995117, 68.624999523162842, 376.379256308078766, 68.624999523162842 ],
					"order" : 11,
					"source" : [ "obj-12", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-225", 0 ],
					"midpoints" : [ 297.735288619995117, 68.624999523162842, 435.202784597873688, 68.624999523162842 ],
					"order" : 8,
					"source" : [ "obj-12", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-226", 0 ],
					"midpoints" : [ 297.735288619995117, 65.624999523162842, 409.427050650119781, 65.624999523162842, 409.427050650119781, 56.624999523162842, 492.555724680423737, 56.624999523162842 ],
					"order" : 6,
					"source" : [ "obj-12", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-227", 0 ],
					"order" : 4,
					"source" : [ "obj-12", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-228", 0 ],
					"midpoints" : [ 297.735288619995117, 149.624999523162842, 491.085136473178864, 149.624999523162842 ],
					"order" : 7,
					"source" : [ "obj-12", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-229", 0 ],
					"midpoints" : [ 297.735288619995117, 149.624999523162842, 433.732196390628815, 149.624999523162842 ],
					"order" : 9,
					"source" : [ "obj-12", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-230", 0 ],
					"midpoints" : [ 297.735288619995117, 149.624999523162842, 376.379256308078766, 149.624999523162842 ],
					"order" : 10,
					"source" : [ "obj-12", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-231", 0 ],
					"midpoints" : [ 297.735288619995117, 65.624999523162842, 409.427050650119781, 65.624999523162842, 409.427050650119781, 56.624999523162842, 549.908664762973785, 56.624999523162842 ],
					"order" : 5,
					"source" : [ "obj-12", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-267", 0 ],
					"order" : 1,
					"source" : [ "obj-12", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-33", 0 ],
					"order" : 2,
					"source" : [ "obj-12", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-350", 0 ],
					"order" : 0,
					"source" : [ "obj-12", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-253", 1 ],
					"midpoints" : [ 914.261896133422852, 1744.324594855308533, 915.404754638671875, 1744.324594855308533 ],
					"source" : [ "obj-126", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-8", 0 ],
					"midpoints" : [ 771.404754638671875, 1693.324594855308533, 771.404754638671875, 1693.324594855308533 ],
					"source" : [ "obj-127", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-257", 0 ],
					"midpoints" : [ 942.833324432373047, 1831.324594855308533, 773.054966926574707, 1831.324594855308533, 773.054966926574707, 1837.324594855308533, 771.404754638671875, 1837.324594855308533 ],
					"source" : [ "obj-129", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-20", 0 ],
					"midpoints" : [ 1190.452369689941406, 1561.324594855308533, 1190.452369689941406, 1561.324594855308533 ],
					"order" : 1,
					"source" : [ "obj-13", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-246", 0 ],
					"midpoints" : [ 1190.452369689941406, 1567.324594855308533, 1257.119035720825195, 1567.324594855308533 ],
					"order" : 0,
					"source" : [ "obj-13", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-22", 0 ],
					"midpoints" : [ 1490.452366828918457, 1663.324594855308533, 1463.054966926574707, 1663.324594855308533, 1463.054966926574707, 1705.324594855308533, 1166.054966926574707, 1705.324594855308533, 1166.054966926574707, 1669.324594855308533, 1190.452369689941406, 1669.324594855308533 ],
					"order" : 2,
					"source" : [ "obj-130", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-223", 0 ],
					"midpoints" : [ 1490.452366828918457, 1663.324594855308533, 1490.452366828918457, 1663.324594855308533 ],
					"order" : 0,
					"source" : [ "obj-130", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-23", 0 ],
					"midpoints" : [ 1490.452366828918457, 1663.324594855308533, 1316.054966926574707, 1663.324594855308533, 1316.054966926574707, 1669.324594855308533, 1314.261892318725586, 1669.324594855308533 ],
					"order" : 1,
					"source" : [ "obj-130", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-257", 0 ],
					"midpoints" : [ 1490.452366828918457, 1663.324594855308533, 1466.054966926574707, 1663.324594855308533, 1466.054966926574707, 1828.324594855308533, 1055.054966926574707, 1828.324594855308533, 1055.054966926574707, 1837.324594855308533, 771.404754638671875, 1837.324594855308533 ],
					"order" : 3,
					"source" : [ "obj-130", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-22", 0 ],
					"midpoints" : [ 1447.595224380493164, 1705.324594855308533, 1166.054966926574707, 1705.324594855308533, 1166.054966926574707, 1669.324594855308533, 1190.452369689941406, 1669.324594855308533 ],
					"order" : 1,
					"source" : [ "obj-131", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-23", 0 ],
					"midpoints" : [ 1447.595224380493164, 1663.324594855308533, 1316.054966926574707, 1663.324594855308533, 1316.054966926574707, 1669.324594855308533, 1314.261892318725586, 1669.324594855308533 ],
					"order" : 0,
					"source" : [ "obj-131", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-22", 0 ],
					"midpoints" : [ 1190.452369689941406, 1663.324594855308533, 1190.452369689941406, 1663.324594855308533 ],
					"source" : [ "obj-132", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-23", 0 ],
					"midpoints" : [ 1314.261892318725586, 1663.324594855308533, 1314.261892318725586, 1663.324594855308533 ],
					"source" : [ "obj-133", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-132", 0 ],
					"midpoints" : [ 1190.452369689941406, 1630.324594855308533, 1190.452369689941406, 1630.324594855308533 ],
					"order" : 1,
					"source" : [ "obj-134", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-133", 0 ],
					"midpoints" : [ 1190.452369689941406, 1636.324594855308533, 1314.261892318725586, 1636.324594855308533 ],
					"order" : 0,
					"source" : [ "obj-134", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-103", 0 ],
					"source" : [ "obj-135", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-154", 0 ],
					"midpoints" : [ 94.115388751029968, 456.0, 94.115388751029968, 456.0 ],
					"source" : [ "obj-139", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-26", 0 ],
					"midpoints" : [ 94.115388751029968, 402.0, 94.115388751029968, 402.0 ],
					"source" : [ "obj-14", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-100", 0 ],
					"midpoints" : [ 891.852924346923828, 179.974127292633057, 891.852924346923828, 179.974127292633057 ],
					"source" : [ "obj-140", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-102", 0 ],
					"midpoints" : [ 777.14704418182373, 179.974127292633057, 777.14704418182373, 179.974127292633057 ],
					"source" : [ "obj-141", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-148", 0 ],
					"midpoints" : [ 642.833327293395996, 1819.083318710327148, 538.071423530578613, 1819.083318710327148 ],
					"source" : [ "obj-142", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-108", 0 ],
					"midpoints" : [ 985.904572367668152, 179.974127292633057, 950.67645263671875, 179.974127292633057 ],
					"source" : [ "obj-143", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-257", 0 ],
					"midpoints" : [ 1071.404751777648926, 1864.324594855308533, 1055.054966926574707, 1864.324594855308533, 1055.054966926574707, 1837.324594855308533, 771.404754638671875, 1837.324594855308533 ],
					"source" : [ "obj-144", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-101", 0 ],
					"midpoints" : [ 833.029396057128906, 179.974127292633057, 833.029396057128906, 179.974127292633057 ],
					"source" : [ "obj-145", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-257", 0 ],
					"midpoints" : [ 1076.166656494140625, 1903.324594855308533, 890.054966926574707, 1903.324594855308533, 890.054966926574707, 1891.324594855308533, 746.054966926574707, 1891.324594855308533, 746.054966926574707, 1837.324594855308533, 771.404754638671875, 1837.324594855308533 ],
					"source" : [ "obj-146", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-148", 0 ],
					"midpoints" : [ 538.071423530578613, 1819.083318710327148, 538.071423530578613, 1819.083318710327148 ],
					"source" : [ "obj-147", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-150", 0 ],
					"midpoints" : [ 538.071423530578613, 1849.083318710327148, 538.071423530578613, 1849.083318710327148 ],
					"source" : [ "obj-148", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-150", 0 ],
					"midpoints" : [ 538.071423530578613, 1786.083318710327148, 508.821426391601562, 1786.083318710327148, 508.821426391601562, 1855.083318710327148, 538.071423530578613, 1855.083318710327148 ],
					"source" : [ "obj-149", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-12", 0 ],
					"source" : [ "obj-15", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-153", 0 ],
					"midpoints" : [ 419.023805618286133, 1819.083318710327148, 309.499997138977051, 1819.083318710327148 ],
					"source" : [ "obj-151", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-153", 0 ],
					"midpoints" : [ 309.499997138977051, 1819.083318710327148, 309.499997138977051, 1819.083318710327148 ],
					"source" : [ "obj-152", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-155", 0 ],
					"midpoints" : [ 309.499997138977051, 1855.083318710327148, 309.499997138977051, 1855.083318710327148 ],
					"source" : [ "obj-153", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-1", 1 ],
					"source" : [ "obj-154", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-158", 0 ],
					"midpoints" : [ 209.499998092651367, 1822.083318710327148, 99.976189613342285, 1822.083318710327148 ],
					"source" : [ "obj-156", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-158", 0 ],
					"midpoints" : [ 99.976189613342285, 1825.083318710327148, 99.976189613342285, 1825.083318710327148 ],
					"source" : [ "obj-157", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-162", 0 ],
					"midpoints" : [ 99.976189613342285, 1852.083318710327148, 99.976189613342285, 1852.083318710327148 ],
					"source" : [ "obj-158", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-271", 0 ],
					"source" : [ "obj-16", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-162", 0 ],
					"midpoints" : [ 99.976189613342285, 1792.083318710327148, 73.821426391601562, 1792.083318710327148, 73.821426391601562, 1855.083318710327148, 99.976189613342285, 1855.083318710327148 ],
					"source" : [ "obj-161", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-165", 0 ],
					"midpoints" : [ 633.309517860412598, 1687.083318710327148, 523.785709381103516, 1687.083318710327148 ],
					"source" : [ "obj-163", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-165", 0 ],
					"midpoints" : [ 523.785709381103516, 1687.083318710327148, 523.785709381103516, 1687.083318710327148 ],
					"source" : [ "obj-164", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-167", 0 ],
					"midpoints" : [ 523.785709381103516, 1714.083318710327148, 523.785709381103516, 1714.083318710327148 ],
					"source" : [ "obj-165", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-167", 0 ],
					"midpoints" : [ 523.785709381103516, 1657.083318710327148, 505.821426391601562, 1657.083318710327148, 505.821426391601562, 1717.083318710327148, 523.785709381103516, 1717.083318710327148 ],
					"source" : [ "obj-166", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-18", 2 ],
					"midpoints" : [ 136.972533702850342, 315.0, 138.895609498023987, 315.0 ],
					"source" : [ "obj-168", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-18", 0 ],
					"midpoints" : [ 104.615388751029968, 312.0, 113.895609498023987, 312.0 ],
					"source" : [ "obj-17", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-19", 0 ],
					"midpoints" : [ 94.115388751029968, 312.0, 94.115388751029968, 312.0 ],
					"source" : [ "obj-17", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-177", 0 ],
					"midpoints" : [ 423.785710334777832, 1687.083318710327148, 314.26190185546875, 1687.083318710327148 ],
					"source" : [ "obj-175", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-177", 0 ],
					"midpoints" : [ 314.26190185546875, 1687.083318710327148, 314.26190185546875, 1687.083318710327148 ],
					"source" : [ "obj-176", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-179", 0 ],
					"midpoints" : [ 314.26190185546875, 1714.083318710327148, 342.833330154418945, 1714.083318710327148 ],
					"source" : [ "obj-177", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-179", 0 ],
					"midpoints" : [ 314.26190185546875, 1657.083318710327148, 295.821426391601562, 1657.083318710327148, 295.821426391601562, 1714.083318710327148, 342.833330154418945, 1714.083318710327148 ],
					"source" : [ "obj-178", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-19", 1 ],
					"midpoints" : [ 113.895609498023987, 342.0, 122.115388751029968, 342.0 ],
					"source" : [ "obj-18", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-142", 0 ],
					"midpoints" : [ 380.928567886352539, 1618.083318710327148, 715.821426391601562, 1618.083318710327148, 715.821426391601562, 1780.083318710327148, 673.821426391601562, 1780.083318710327148, 673.821426391601562, 1786.083318710327148, 642.833327293395996, 1786.083318710327148 ],
					"order" : 0,
					"source" : [ "obj-180", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-151", 0 ],
					"midpoints" : [ 380.928567886352539, 1618.083318710327148, 496.821426391601562, 1618.083318710327148, 496.821426391601562, 1660.083318710327148, 499.821426391601562, 1660.083318710327148, 499.821426391601562, 1696.083318710327148, 460.821426391601562, 1696.083318710327148, 460.821426391601562, 1783.083318710327148, 448.821426391601562, 1783.083318710327148, 448.821426391601562, 1792.083318710327148, 419.023805618286133, 1792.083318710327148 ],
					"order" : 4,
					"source" : [ "obj-180", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-156", 0 ],
					"midpoints" : [ 380.928567886352539, 1618.083318710327148, 286.821426391601562, 1618.083318710327148, 286.821426391601562, 1660.083318710327148, 292.821426391601562, 1660.083318710327148, 292.821426391601562, 1696.083318710327148, 256.821426391601562, 1696.083318710327148, 256.821426391601562, 1786.083318710327148, 238.821426391601562, 1786.083318710327148, 238.821426391601562, 1792.083318710327148, 209.499998092651367, 1792.083318710327148 ],
					"order" : 6,
					"source" : [ "obj-180", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-163", 0 ],
					"midpoints" : [ 380.928567886352539, 1618.083318710327148, 670.821426391601562, 1618.083318710327148, 670.821426391601562, 1657.083318710327148, 633.309517860412598, 1657.083318710327148 ],
					"order" : 1,
					"source" : [ "obj-180", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-175", 0 ],
					"midpoints" : [ 380.928567886352539, 1618.083318710327148, 460.821426391601562, 1618.083318710327148, 460.821426391601562, 1660.083318710327148, 423.785710334777832, 1660.083318710327148 ],
					"order" : 3,
					"source" : [ "obj-180", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-181", 0 ],
					"midpoints" : [ 380.928567886352539, 1618.083318710327148, 256.821426391601562, 1618.083318710327148, 256.821426391601562, 1657.083318710327148, 214.261902809143066, 1657.083318710327148 ],
					"order" : 5,
					"source" : [ "obj-180", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-265", 1 ],
					"midpoints" : [ 380.928567886352539, 1558.083318710327148, 391.821426391601562, 1558.083318710327148, 391.821426391601562, 1555.083318710327148, 447.999996185302734, 1555.083318710327148 ],
					"order" : 2,
					"source" : [ "obj-180", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-183", 0 ],
					"midpoints" : [ 214.261902809143066, 1687.083318710327148, 109.499999046325684, 1687.083318710327148 ],
					"source" : [ "obj-181", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-183", 0 ],
					"midpoints" : [ 109.499999046325684, 1687.083318710327148, 109.499999046325684, 1687.083318710327148 ],
					"source" : [ "obj-182", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-200", 0 ],
					"midpoints" : [ 109.499999046325684, 1714.083318710327148, 109.499999046325684, 1714.083318710327148 ],
					"source" : [ "obj-183", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-47", 0 ],
					"midpoints" : [ 376.379256308078766, 140.624999523162842, 352.427050650119781, 140.624999523162842, 352.427050650119781, 239.624999523162842, 376.379256308078766, 239.624999523162842 ],
					"source" : [ "obj-184", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-187", 0 ],
					"source" : [ "obj-186", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-192", 0 ],
					"source" : [ "obj-187", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-196", 0 ],
					"source" : [ "obj-187", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-190", 0 ],
					"source" : [ "obj-188", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-265", 0 ],
					"midpoints" : [ 409.499996185302734, 1558.083318710327148, 409.499996185302734, 1558.083318710327148 ],
					"source" : [ "obj-189", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-14", 0 ],
					"midpoints" : [ 94.115388751029968, 375.0, 94.115388751029968, 375.0 ],
					"source" : [ "obj-19", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-197", 0 ],
					"source" : [ "obj-190", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-186", 0 ],
					"order" : 1,
					"source" : [ "obj-192", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-188", 0 ],
					"order" : 0,
					"source" : [ "obj-192", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-265", 2 ],
					"midpoints" : [ 495.21428108215332, 1555.083318710327148, 486.499996185302734, 1555.083318710327148 ],
					"source" : [ "obj-193", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-215", 0 ],
					"source" : [ "obj-196", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-215", 1 ],
					"source" : [ "obj-197", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-200", 0 ],
					"midpoints" : [ 109.499999046325684, 1657.083318710327148, 82.821426391601562, 1657.083318710327148, 82.821426391601562, 1717.083318710327148, 109.499999046325684, 1717.083318710327148 ],
					"source" : [ "obj-199", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-29", 0 ],
					"midpoints" : [ 611.673369467258453, 137.624999523162842, 611.673369467258453, 137.624999523162842 ],
					"source" : [ "obj-2", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-134", 1 ],
					"midpoints" : [ 1190.452369689941406, 1600.324594855308533, 1333.452369689941406, 1600.324594855308533 ],
					"source" : [ "obj-20", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-206", 0 ],
					"midpoints" : [ 981.261896133422852, 1615.324594855308533, 992.054966926574707, 1615.324594855308533, 992.054966926574707, 1642.324594855308533, 1038.071418762207031, 1642.324594855308533 ],
					"order" : 0,
					"source" : [ "obj-205", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-207", 0 ],
					"midpoints" : [ 914.261896133422852, 1615.324594855308533, 1009.499990463256836, 1615.324594855308533 ],
					"order" : 0,
					"source" : [ "obj-205", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-256", 1 ],
					"midpoints" : [ 981.261896133422852, 1609.324594855308533, 981.261896133422852, 1609.324594855308533 ],
					"order" : 1,
					"source" : [ "obj-205", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-256", 0 ],
					"midpoints" : [ 914.261896133422852, 1609.324594855308533, 914.261896133422852, 1609.324594855308533 ],
					"order" : 1,
					"source" : [ "obj-205", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-205", 0 ],
					"midpoints" : [ 167.906230121850967, 1267.085106372833252, 743.238097190856934, 1267.085106372833252, 743.238097190856934, 1138.085106372833252, 887.238097190856934, 1138.085106372833252, 887.238097190856934, 1093.085106372833252, 914.261896133422852, 1093.085106372833252 ],
					"source" : [ "obj-209", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-208", 0 ],
					"midpoints" : [ 114.261903762817383, 1585.083318710327148, 114.261903762817383, 1585.083318710327148 ],
					"source" : [ "obj-209", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-47", 0 ],
					"midpoints" : [ 435.202784597873688, 140.624999523162842, 418.427050650119781, 140.624999523162842, 418.427050650119781, 239.624999523162842, 376.379256308078766, 239.624999523162842 ],
					"source" : [ "obj-213", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-216", 0 ],
					"source" : [ "obj-214", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-214", 0 ],
					"source" : [ "obj-215", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-205", 0 ],
					"source" : [ "obj-216", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-47", 0 ],
					"midpoints" : [ 492.555724680423737, 140.624999523162842, 478.427050650119781, 140.624999523162842, 478.427050650119781, 239.624999523162842, 376.379256308078766, 239.624999523162842 ],
					"source" : [ "obj-218", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-186", 0 ],
					"order" : 1,
					"source" : [ "obj-219", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-188", 0 ],
					"order" : 0,
					"source" : [ "obj-219", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-16", 0 ],
					"source" : [ "obj-22", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-170", 1 ],
					"source" : [ "obj-22", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-187", 0 ],
					"order" : 1,
					"source" : [ "obj-221", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-190", 0 ],
					"order" : 0,
					"source" : [ "obj-221", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-47", 0 ],
					"midpoints" : [ 546.408664762973785, 239.624999523162842, 376.379256308078766, 239.624999523162842 ],
					"source" : [ "obj-222", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-184", 0 ],
					"midpoints" : [ 376.379256308078766, 110.624999523162842, 376.379256308078766, 110.624999523162842 ],
					"order" : 1,
					"source" : [ "obj-224", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-2", 0 ],
					"midpoints" : [ 376.379256308078766, 110.624999523162842, 409.427050650119781, 110.624999523162842, 409.427050650119781, 56.624999523162842, 611.673369467258453, 56.624999523162842 ],
					"order" : 0,
					"source" : [ "obj-224", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-2", 1 ],
					"midpoints" : [ 435.202784597873688, 110.624999523162842, 469.427050650119781, 110.624999523162842, 469.427050650119781, 56.624999523162842, 622.173369467258453, 56.624999523162842 ],
					"order" : 0,
					"source" : [ "obj-225", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-213", 0 ],
					"midpoints" : [ 435.202784597873688, 110.624999523162842, 435.202784597873688, 110.624999523162842 ],
					"order" : 1,
					"source" : [ "obj-225", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-2", 2 ],
					"midpoints" : [ 492.555724680423737, 110.624999523162842, 589.427050650119781, 110.624999523162842, 589.427050650119781, 98.624999523162842, 632.673369467258453, 98.624999523162842 ],
					"order" : 0,
					"source" : [ "obj-226", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-218", 0 ],
					"midpoints" : [ 492.555724680423737, 110.624999523162842, 492.555724680423737, 110.624999523162842 ],
					"order" : 1,
					"source" : [ "obj-226", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-222", 0 ],
					"source" : [ "obj-227", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-2", 6 ],
					"midpoints" : [ 491.085136473178864, 191.624999523162842, 517.427050650119781, 191.624999523162842, 517.427050650119781, 149.624999523162842, 598.427050650119781, 149.624999523162842, 598.427050650119781, 98.624999523162842, 674.673369467258453, 98.624999523162842 ],
					"order" : 0,
					"source" : [ "obj-228", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-232", 0 ],
					"midpoints" : [ 491.085136473178864, 191.624999523162842, 491.085136473178864, 191.624999523162842 ],
					"order" : 1,
					"source" : [ "obj-228", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-2", 5 ],
					"midpoints" : [ 433.732196390628815, 191.624999523162842, 460.427050650119781, 191.624999523162842, 460.427050650119781, 149.624999523162842, 598.427050650119781, 149.624999523162842, 598.427050650119781, 98.624999523162842, 664.173369467258453, 98.624999523162842 ],
					"order" : 0,
					"source" : [ "obj-229", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-233", 0 ],
					"midpoints" : [ 433.732196390628815, 191.624999523162842, 433.732196390628815, 191.624999523162842 ],
					"order" : 1,
					"source" : [ "obj-229", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-2", 4 ],
					"midpoints" : [ 376.379256308078766, 191.624999523162842, 352.427050650119781, 191.624999523162842, 352.427050650119781, 56.624999523162842, 653.673369467258453, 56.624999523162842 ],
					"order" : 0,
					"source" : [ "obj-230", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-234", 0 ],
					"midpoints" : [ 376.379256308078766, 191.624999523162842, 376.379256308078766, 191.624999523162842 ],
					"order" : 1,
					"source" : [ "obj-230", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-2", 3 ],
					"midpoints" : [ 549.908664762973785, 110.624999523162842, 589.427050650119781, 110.624999523162842, 589.427050650119781, 98.624999523162842, 643.173369467258453, 98.624999523162842 ],
					"order" : 0,
					"source" : [ "obj-231", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-235", 0 ],
					"midpoints" : [ 549.908664762973785, 110.624999523162842, 549.908664762973785, 110.624999523162842 ],
					"order" : 1,
					"source" : [ "obj-231", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-47", 0 ],
					"midpoints" : [ 491.085136473178864, 239.624999523162842, 376.379256308078766, 239.624999523162842 ],
					"source" : [ "obj-232", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-47", 0 ],
					"midpoints" : [ 433.732196390628815, 239.624999523162842, 376.379256308078766, 239.624999523162842 ],
					"source" : [ "obj-233", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-47", 0 ],
					"midpoints" : [ 376.379256308078766, 251.624999523162842, 376.379256308078766, 251.624999523162842 ],
					"source" : [ "obj-234", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-47", 0 ],
					"midpoints" : [ 549.908664762973785, 140.624999523162842, 535.427050650119781, 140.624999523162842, 535.427050650119781, 239.624999523162842, 376.379256308078766, 239.624999523162842 ],
					"source" : [ "obj-235", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-187", 0 ],
					"order" : 1,
					"source" : [ "obj-236", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-190", 0 ],
					"order" : 0,
					"source" : [ "obj-236", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-187", 0 ],
					"order" : 1,
					"source" : [ "obj-237", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-190", 0 ],
					"order" : 0,
					"source" : [ "obj-237", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-22", 0 ],
					"source" : [ "obj-244", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-180", 0 ],
					"midpoints" : [ 447.595233917236328, 1555.083318710327148, 424.821426391601562, 1555.083318710327148, 424.821426391601562, 1519.083318710327148, 380.928567886352539, 1519.083318710327148 ],
					"source" : [ "obj-247", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-155", 0 ],
					"midpoints" : [ 309.499997138977051, 1792.083318710327148, 292.821426391601562, 1792.083318710327148, 292.821426391601562, 1855.083318710327148, 309.499997138977051, 1855.083318710327148 ],
					"source" : [ "obj-248", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-9", 0 ],
					"midpoints" : [ 87.625, 24.0, 87.625, 24.0 ],
					"source" : [ "obj-25", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-38", 0 ],
					"midpoints" : [ 771.404754638671875, 1771.324594855308533, 771.404754638671875, 1771.324594855308533 ],
					"source" : [ "obj-253", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-42", 0 ],
					"midpoints" : [ 766.642849922180176, 1951.324594855308533, 766.642849922180176, 1951.324594855308533 ],
					"source" : [ "obj-254", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-3", 0 ],
					"midpoints" : [ 914.261896133422852, 1672.324594855308533, 914.261896133422852, 1672.324594855308533 ],
					"source" : [ "obj-255", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-255", 0 ],
					"midpoints" : [ 914.261896133422852, 1642.324594855308533, 914.261896133422852, 1642.324594855308533 ],
					"source" : [ "obj-256", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-254", 0 ],
					"midpoints" : [ 771.404754638671875, 1894.324594855308533, 752.054966926574707, 1894.324594855308533, 752.054966926574707, 1924.324594855308533, 766.642849922180176, 1924.324594855308533 ],
					"order" : 1,
					"source" : [ "obj-257", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-41", 1 ],
					"midpoints" : [ 771.404754638671875, 1894.324594855308533, 878.642849922180176, 1894.324594855308533 ],
					"order" : 0,
					"source" : [ "obj-257", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-139", 0 ],
					"midpoints" : [ 94.115388751029968, 426.0, 94.115388751029968, 426.0 ],
					"source" : [ "obj-26", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-147", 0 ],
					"midpoints" : [ 482.999996185302734, 1648.083318710327148, 499.821426391601562, 1648.083318710327148, 499.821426391601562, 1783.083318710327148, 508.821426391601562, 1783.083318710327148, 508.821426391601562, 1789.083318710327148, 538.071423530578613, 1789.083318710327148 ],
					"source" : [ "obj-265", 7 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-149", 0 ],
					"midpoints" : [ 419.999996185302734, 1618.083318710327148, 496.821426391601562, 1618.083318710327148, 496.821426391601562, 1660.083318710327148, 499.821426391601562, 1660.083318710327148, 499.821426391601562, 1750.083318710327148, 538.071423530578613, 1750.083318710327148 ],
					"source" : [ "obj-265", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-152", 0 ],
					"midpoints" : [ 493.499996185302734, 1648.083318710327148, 499.821426391601562, 1648.083318710327148, 499.821426391601562, 1696.083318710327148, 460.821426391601562, 1696.083318710327148, 460.821426391601562, 1756.083318710327148, 283.821426391601562, 1756.083318710327148, 283.821426391601562, 1792.083318710327148, 309.499997138977051, 1792.083318710327148 ],
					"source" : [ "obj-265", 8 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-157", 0 ],
					"midpoints" : [ 503.999996185302734, 1618.083318710327148, 73.821426391601562, 1618.083318710327148, 73.821426391601562, 1795.083318710327148, 99.976189613342285, 1795.083318710327148 ],
					"source" : [ "obj-265", 9 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-161", 0 ],
					"midpoints" : [ 440.999996185302734, 1618.083318710327148, 82.821426391601562, 1618.083318710327148, 82.821426391601562, 1756.083318710327148, 99.976189613342285, 1756.083318710327148 ],
					"source" : [ "obj-265", 3 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-164", 0 ],
					"midpoints" : [ 514.499996185302734, 1618.083318710327148, 496.821426391601562, 1618.083318710327148, 496.821426391601562, 1657.083318710327148, 523.785709381103516, 1657.083318710327148 ],
					"source" : [ "obj-265", 10 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-166", 0 ],
					"midpoints" : [ 451.499996185302734, 1618.083318710327148, 523.785709381103516, 1618.083318710327148 ],
					"source" : [ "obj-265", 4 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-176", 0 ],
					"midpoints" : [ 524.999996185302734, 1618.083318710327148, 286.821426391601562, 1618.083318710327148, 286.821426391601562, 1657.083318710327148, 314.26190185546875, 1657.083318710327148 ],
					"source" : [ "obj-265", 11 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-178", 0 ],
					"midpoints" : [ 461.999996185302734, 1618.083318710327148, 314.26190185546875, 1618.083318710327148 ],
					"source" : [ "obj-265", 5 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-182", 0 ],
					"midpoints" : [ 472.499996185302734, 1618.083318710327148, 82.821426391601562, 1618.083318710327148, 82.821426391601562, 1660.083318710327148, 109.499999046325684, 1660.083318710327148 ],
					"source" : [ "obj-265", 6 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-199", 0 ],
					"midpoints" : [ 409.499996185302734, 1618.083318710327148, 106.821426391601562, 1618.083318710327148, 106.821426391601562, 1630.083318710327148, 109.499999046325684, 1630.083318710327148 ],
					"source" : [ "obj-265", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-248", 0 ],
					"midpoints" : [ 430.499996185302734, 1618.083318710327148, 286.821426391601562, 1618.083318710327148, 286.821426391601562, 1660.083318710327148, 292.821426391601562, 1660.083318710327148, 292.821426391601562, 1756.083318710327148, 309.499997138977051, 1756.083318710327148 ],
					"source" : [ "obj-265", 2 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-284", 7 ],
					"midpoints" : [ 1856.43875789642334, 158.987484931945801, 2108.951078653335571, 158.987484931945801, 2108.951078653335571, 140.987484931945801, 2225.951078653335571, 140.987484931945801, 2225.951078653335571, 74.987484931945801, 2217.693857192993164, 74.987484931945801 ],
					"order" : 0,
					"source" : [ "obj-267", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-285", 0 ],
					"order" : 3,
					"source" : [ "obj-267", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-286", 0 ],
					"order" : 6,
					"source" : [ "obj-267", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-287", 0 ],
					"order" : 7,
					"source" : [ "obj-267", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-288", 0 ],
					"order" : 2,
					"source" : [ "obj-267", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-293", 0 ],
					"order" : 1,
					"source" : [ "obj-267", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-294", 0 ],
					"order" : 4,
					"source" : [ "obj-267", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-295", 0 ],
					"order" : 5,
					"source" : [ "obj-267", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-296", 0 ],
					"order" : 8,
					"source" : [ "obj-267", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-273", 0 ],
					"source" : [ "obj-268", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-290", 0 ],
					"midpoints" : [ 1964.602022171020508, 188.797656536102295, 1964.602022171020508, 188.797656536102295 ],
					"source" : [ "obj-274", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-297", 0 ],
					"midpoints" : [ 2129.724888443946838, 188.797656536102295, 2088.713649213314056, 188.797656536102295 ],
					"source" : [ "obj-275", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-291", 0 ],
					"midpoints" : [ 1907.459165573120117, 188.797656536102295, 1907.459165573120117, 188.797656536102295 ],
					"source" : [ "obj-278", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-289", 0 ],
					"midpoints" : [ 2021.744878768920898, 188.797656536102295, 2021.744878768920898, 188.797656536102295 ],
					"source" : [ "obj-279", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-10", 0 ],
					"source" : [ "obj-28", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-292", 0 ],
					"source" : [ "obj-280", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-298", 0 ],
					"source" : [ "obj-281", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-299", 0 ],
					"source" : [ "obj-282", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-300", 0 ],
					"midpoints" : [ 1907.459165573120117, 77.987484931945801, 1907.459165573120117, 77.987484931945801 ],
					"source" : [ "obj-283", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-279", 0 ],
					"order" : 1,
					"source" : [ "obj-285", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-284", 6 ],
					"midpoints" : [ 2026.744878768920898, 158.987484931945801, 2048.951078653335571, 158.987484931945801, 2048.951078653335571, 116.987484931945801, 2129.951078653335571, 116.987484931945801, 2129.951078653335571, 65.987484931945801, 2207.193857192993164, 65.987484931945801 ],
					"order" : 0,
					"source" : [ "obj-285", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-274", 0 ],
					"order" : 1,
					"source" : [ "obj-286", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-284", 5 ],
					"midpoints" : [ 1964.602022171020508, 158.987484931945801, 1991.951078653335571, 158.987484931945801, 1991.951078653335571, 116.987484931945801, 2129.951078653335571, 116.987484931945801, 2129.951078653335571, 65.987484931945801, 2196.693857192993164, 65.987484931945801 ],
					"order" : 0,
					"source" : [ "obj-286", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-278", 0 ],
					"order" : 1,
					"source" : [ "obj-287", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-284", 4 ],
					"midpoints" : [ 1907.459165573120117, 158.987484931945801, 1883.951078653335571, 158.987484931945801, 1883.951078653335571, 23.987484931945801, 2186.193857192993164, 23.987484931945801 ],
					"order" : 0,
					"source" : [ "obj-287", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-280", 0 ],
					"midpoints" : [ 2080.92855167388916, 77.987484931945801, 2080.92855167388916, 77.987484931945801 ],
					"order" : 1,
					"source" : [ "obj-288", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-284", 3 ],
					"midpoints" : [ 2080.92855167388916, 77.987484931945801, 2120.951078653335571, 77.987484931945801, 2120.951078653335571, 65.987484931945801, 2175.693857192993164, 65.987484931945801 ],
					"order" : 0,
					"source" : [ "obj-288", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-302", 0 ],
					"midpoints" : [ 2021.744878768920898, 236.797656536102295, 1907.459165573120117, 236.797656536102295 ],
					"source" : [ "obj-289", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-302", 0 ],
					"midpoints" : [ 1964.602022171020508, 236.797656536102295, 1907.459165573120117, 236.797656536102295 ],
					"source" : [ "obj-290", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-302", 0 ],
					"midpoints" : [ 1907.459165573120117, 248.797656536102295, 1907.459165573120117, 248.797656536102295 ],
					"source" : [ "obj-291", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-302", 0 ],
					"midpoints" : [ 2080.92855167388916, 107.987484931945801, 2066.951078653335571, 107.987484931945801, 2066.951078653335571, 206.987484931945801, 1907.459165573120117, 206.987484931945801 ],
					"source" : [ "obj-292", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-275", 0 ],
					"source" : [ "obj-293", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-281", 0 ],
					"midpoints" : [ 2023.78569507598877, 77.987484931945801, 2023.78569507598877, 77.987484931945801 ],
					"order" : 1,
					"source" : [ "obj-294", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-284", 2 ],
					"midpoints" : [ 2023.78569507598877, 77.987484931945801, 2120.951078653335571, 77.987484931945801, 2120.951078653335571, 65.987484931945801, 2165.193857192993164, 65.987484931945801 ],
					"order" : 0,
					"source" : [ "obj-294", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-282", 0 ],
					"order" : 1,
					"source" : [ "obj-295", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-284", 1 ],
					"order" : 0,
					"source" : [ "obj-295", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-283", 0 ],
					"order" : 1,
					"source" : [ "obj-296", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-284", 0 ],
					"midpoints" : [ 1907.459165573120117, 77.987484931945801, 1940.951078653335571, 77.987484931945801, 1940.951078653335571, 23.987484931945801, 2144.193857192993164, 23.987484931945801 ],
					"order" : 0,
					"source" : [ "obj-296", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-302", 0 ],
					"midpoints" : [ 2088.713649213314056, 236.797656536102295, 1907.459165573120117, 236.797656536102295 ],
					"source" : [ "obj-297", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-302", 0 ],
					"midpoints" : [ 2021.744878768920898, 107.987484931945801, 2009.951078653335571, 107.987484931945801, 2009.951078653335571, 206.987484931945801, 1907.459165573120117, 206.987484931945801 ],
					"source" : [ "obj-298", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-302", 0 ],
					"midpoints" : [ 1964.602022171020508, 107.987484931945801, 1949.951078653335571, 107.987484931945801, 1949.951078653335571, 206.987484931945801, 1907.459165573120117, 206.987484931945801 ],
					"source" : [ "obj-299", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-126", 0 ],
					"midpoints" : [ 914.261896133422852, 1702.324594855308533, 914.261896133422852, 1702.324594855308533 ],
					"source" : [ "obj-3", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-302", 0 ],
					"midpoints" : [ 1907.459165573120117, 107.987484931945801, 1883.951078653335571, 107.987484931945801, 1883.951078653335571, 206.987484931945801, 1907.459165573120117, 206.987484931945801 ],
					"source" : [ "obj-300", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-331", 0 ],
					"midpoints" : [ 1905.418349266052246, 662.823529243469238, 1532.822325229644775, 662.823529243469238, 1532.822325229644775, 524.823529243469238, 1497.255087852478027, 524.823529243469238 ],
					"source" : [ "obj-301", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-301", 0 ],
					"midpoints" : [ 1907.459165573120117, 290.59130072593689, 1905.418349266052246, 290.59130072593689 ],
					"source" : [ "obj-302", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-304", 0 ],
					"midpoints" : [ 1142.153050422668457, 215.823529243469238, 1142.153050422668457, 215.823529243469238 ],
					"source" : [ "obj-303", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-305", 0 ],
					"midpoints" : [ 1142.153050422668457, 242.823529243469238, 1127.822325229644775, 242.823529243469238, 1127.822325229644775, 215.823529243469238, 1310.822325229644775, 215.823529243469238, 1310.822325229644775, 221.823529243469238, 1315.6224365234375, 221.823529243469238 ],
					"source" : [ "obj-304", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-306", 2 ],
					"midpoints" : [ 1315.6224365234375, 251.823529243469238, 1361.822325229644775, 251.823529243469238, 1361.822325229644775, 185.823529243469238, 1351.704069137573242, 185.823529243469238 ],
					"source" : [ "obj-305", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-314", 0 ],
					"midpoints" : [ 1319.704069137573242, 215.823529243469238, 1187.822325229644775, 215.823529243469238, 1187.822325229644775, 29.823529243469238, 1217.663253784179688, 29.823529243469238 ],
					"source" : [ "obj-306", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-342", 0 ],
					"midpoints" : [ 1351.704069137573242, 275.823529243469238, 1241.822325229644775, 275.823529243469238, 1241.822325229644775, 266.823529243469238, 1225.826519012451172, 266.823529243469238 ],
					"source" : [ "obj-306", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-310", 7 ],
					"midpoints" : [ 1571.214271545410156, 65.44852876663208, 1657.249375879764557, 65.44852876663208, 1657.249375879764557, 122.44852876663208, 1666.249375879764557, 122.44852876663208, 1666.249375879764557, 155.44852876663208, 1680.928555488586426, 155.44852876663208, 1680.928555488586426, 200.44852876663208, 1708.249375879764557, 200.44852876663208, 1708.249375879764557, 182.44852876663208, 1825.249375879764557, 182.44852876663208, 1825.249375879764557, 116.44852876663208, 1815.653044700622559, 116.44852876663208 ],
					"order" : 0,
					"source" : [ "obj-308", 7 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-315", 0 ],
					"midpoints" : [ 1560.357128688267267, 65.44852876663208, 1600.249375879764557, 65.44852876663208, 1600.249375879764557, 122.44852876663208, 1609.249375879764557, 122.44852876663208, 1609.249375879764557, 155.44852876663208, 1621.744882583618164, 155.44852876663208 ],
					"source" : [ "obj-308", 6 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-316", 0 ],
					"midpoints" : [ 1549.499985831124377, 65.44852876663208, 1540.249375879764557, 65.44852876663208, 1540.249375879764557, 122.44852876663208, 1552.249375879764557, 122.44852876663208, 1552.249375879764557, 155.44852876663208, 1564.602025985717773, 155.44852876663208 ],
					"source" : [ "obj-308", 5 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-317", 0 ],
					"midpoints" : [ 1538.642842973981487, 65.44852876663208, 1483.249375879764557, 65.44852876663208, 1483.249375879764557, 158.44852876663208, 1507.459169387817383, 158.44852876663208 ],
					"source" : [ "obj-308", 4 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-318", 0 ],
					"midpoints" : [ 1527.785700116838825, 65.44852876663208, 1680.928555488586426, 65.44852876663208 ],
					"source" : [ "obj-308", 3 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-324", 0 ],
					"midpoints" : [ 1516.928557259695935, 65.44852876663208, 1623.785698890686035, 65.44852876663208 ],
					"source" : [ "obj-308", 2 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-325", 0 ],
					"midpoints" : [ 1506.071414402553046, 65.44852876663208, 1566.642842292785645, 65.44852876663208 ],
					"source" : [ "obj-308", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-326", 0 ],
					"midpoints" : [ 1495.214271545410156, 65.44852876663208, 1507.459169387817383, 65.44852876663208 ],
					"source" : [ "obj-308", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-327", 0 ],
					"midpoints" : [ 1571.214271545410156, 65.44852876663208, 1657.249375879764557, 65.44852876663208, 1657.249375879764557, 122.44852876663208, 1666.249375879764557, 122.44852876663208, 1666.249375879764557, 155.44852876663208, 1680.928555488586426, 155.44852876663208, 1680.928555488586426, 200.44852876663208, 1695.45519083738327, 200.44852876663208 ],
					"order" : 1,
					"source" : [ "obj-308", 7 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-309", 0 ],
					"midpoints" : [ 1742.153044700622559, 146.44852876663208, 1742.153044700622559, 146.44852876663208 ],
					"source" : [ "obj-310", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-310", 7 ],
					"midpoints" : [ 1427.86733341217041, 74.44852876663208, 1540.249375879764557, 74.44852876663208, 1540.249375879764557, 65.44852876663208, 1657.249375879764557, 65.44852876663208, 1657.249375879764557, 122.44852876663208, 1666.249375879764557, 122.44852876663208, 1666.249375879764557, 155.44852876663208, 1680.928555488586426, 155.44852876663208, 1680.928555488586426, 200.44852876663208, 1708.249375879764557, 200.44852876663208, 1708.249375879764557, 182.44852876663208, 1825.249375879764557, 182.44852876663208, 1825.249375879764557, 116.44852876663208, 1815.653044700622559, 116.44852876663208 ],
					"order" : 0,
					"source" : [ "obj-311", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-315", 0 ],
					"midpoints" : [ 1427.86733341217041, 158.44852876663208, 1621.744882583618164, 158.44852876663208 ],
					"order" : 4,
					"source" : [ "obj-311", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-316", 0 ],
					"midpoints" : [ 1427.86733341217041, 158.44852876663208, 1564.602025985717773, 158.44852876663208 ],
					"order" : 6,
					"source" : [ "obj-311", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-317", 0 ],
					"midpoints" : [ 1427.86733341217041, 158.44852876663208, 1507.459169387817383, 158.44852876663208 ],
					"order" : 7,
					"source" : [ "obj-311", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-318", 0 ],
					"midpoints" : [ 1427.86733341217041, 74.44852876663208, 1540.249375879764557, 74.44852876663208, 1540.249375879764557, 65.44852876663208, 1680.928555488586426, 65.44852876663208 ],
					"order" : 2,
					"source" : [ "obj-311", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-323", 0 ],
					"order" : 1,
					"source" : [ "obj-311", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-324", 0 ],
					"midpoints" : [ 1427.86733341217041, 74.44852876663208, 1540.249375879764557, 74.44852876663208, 1540.249375879764557, 65.44852876663208, 1623.785698890686035, 65.44852876663208 ],
					"order" : 3,
					"source" : [ "obj-311", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-325", 0 ],
					"midpoints" : [ 1427.86733341217041, 77.44852876663208, 1566.642842292785645, 77.44852876663208 ],
					"order" : 5,
					"source" : [ "obj-311", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-326", 0 ],
					"midpoints" : [ 1427.86733341217041, 77.44852876663208, 1507.459169387817383, 77.44852876663208 ],
					"order" : 8,
					"source" : [ "obj-311", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-306", 1 ],
					"midpoints" : [ 1217.663253784179688, 182.823529243469238, 1335.704069137573242, 182.823529243469238 ],
					"source" : [ "obj-314", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-310", 6 ],
					"midpoints" : [ 1621.744882583618164, 200.44852876663208, 1648.249375879764557, 200.44852876663208, 1648.249375879764557, 158.44852876663208, 1729.249375879764557, 158.44852876663208, 1729.249375879764557, 107.44852876663208, 1805.153044700622559, 107.44852876663208 ],
					"order" : 0,
					"source" : [ "obj-315", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-319", 0 ],
					"midpoints" : [ 1621.744882583618164, 200.44852876663208, 1621.744882583618164, 200.44852876663208 ],
					"order" : 1,
					"source" : [ "obj-315", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-310", 5 ],
					"midpoints" : [ 1564.602025985717773, 200.44852876663208, 1591.249375879764557, 200.44852876663208, 1591.249375879764557, 158.44852876663208, 1729.249375879764557, 158.44852876663208, 1729.249375879764557, 107.44852876663208, 1794.653044700622559, 107.44852876663208 ],
					"order" : 0,
					"source" : [ "obj-316", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-320", 0 ],
					"midpoints" : [ 1564.602025985717773, 200.44852876663208, 1564.602025985717773, 200.44852876663208 ],
					"order" : 1,
					"source" : [ "obj-316", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-310", 4 ],
					"midpoints" : [ 1507.459169387817383, 200.44852876663208, 1483.249375879764557, 200.44852876663208, 1483.249375879764557, 65.44852876663208, 1784.153044700622559, 65.44852876663208 ],
					"order" : 0,
					"source" : [ "obj-317", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-321", 0 ],
					"midpoints" : [ 1507.459169387817383, 200.44852876663208, 1507.459169387817383, 200.44852876663208 ],
					"order" : 1,
					"source" : [ "obj-317", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-310", 3 ],
					"midpoints" : [ 1680.928555488586426, 119.44852876663208, 1720.249375879764557, 119.44852876663208, 1720.249375879764557, 107.44852876663208, 1773.653044700622559, 107.44852876663208 ],
					"order" : 0,
					"source" : [ "obj-318", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-322", 0 ],
					"midpoints" : [ 1680.928555488586426, 119.44852876663208, 1680.928555488586426, 119.44852876663208 ],
					"order" : 1,
					"source" : [ "obj-318", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-333", 0 ],
					"midpoints" : [ 1621.744882583618164, 248.44852876663208, 1507.459169387817383, 248.44852876663208 ],
					"source" : [ "obj-319", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-333", 0 ],
					"midpoints" : [ 1564.602025985717773, 248.44852876663208, 1507.459169387817383, 248.44852876663208 ],
					"source" : [ "obj-320", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-333", 0 ],
					"midpoints" : [ 1507.459169387817383, 260.44852876663208, 1507.459169387817383, 260.44852876663208 ],
					"source" : [ "obj-321", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-333", 0 ],
					"midpoints" : [ 1680.928555488586426, 149.44852876663208, 1666.249375879764557, 149.44852876663208, 1666.249375879764557, 248.44852876663208, 1507.459169387817383, 248.44852876663208 ],
					"source" : [ "obj-322", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-327", 0 ],
					"source" : [ "obj-323", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-310", 2 ],
					"midpoints" : [ 1623.785698890686035, 119.44852876663208, 1720.249375879764557, 119.44852876663208, 1720.249375879764557, 107.44852876663208, 1763.153044700622559, 107.44852876663208 ],
					"order" : 0,
					"source" : [ "obj-324", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-328", 0 ],
					"midpoints" : [ 1623.785698890686035, 119.44852876663208, 1623.785698890686035, 119.44852876663208 ],
					"order" : 1,
					"source" : [ "obj-324", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-310", 1 ],
					"midpoints" : [ 1566.642842292785645, 119.44852876663208, 1600.249375879764557, 119.44852876663208, 1600.249375879764557, 65.44852876663208, 1752.653044700622559, 65.44852876663208 ],
					"order" : 0,
					"source" : [ "obj-325", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-329", 0 ],
					"midpoints" : [ 1566.642842292785645, 119.44852876663208, 1566.642842292785645, 119.44852876663208 ],
					"order" : 1,
					"source" : [ "obj-325", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-310", 0 ],
					"midpoints" : [ 1507.459169387817383, 119.44852876663208, 1540.249375879764557, 119.44852876663208, 1540.249375879764557, 65.44852876663208, 1742.153044700622559, 65.44852876663208 ],
					"order" : 0,
					"source" : [ "obj-326", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-330", 0 ],
					"midpoints" : [ 1507.459169387817383, 119.44852876663208, 1507.459169387817383, 119.44852876663208 ],
					"order" : 1,
					"source" : [ "obj-326", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-333", 0 ],
					"midpoints" : [ 1695.45519083738327, 248.44852876663208, 1507.459169387817383, 248.44852876663208 ],
					"source" : [ "obj-327", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-333", 0 ],
					"midpoints" : [ 1623.785698890686035, 149.44852876663208, 1609.249375879764557, 149.44852876663208, 1609.249375879764557, 248.44852876663208, 1507.459169387817383, 248.44852876663208 ],
					"source" : [ "obj-328", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-333", 0 ],
					"midpoints" : [ 1566.642842292785645, 149.44852876663208, 1549.249375879764557, 149.44852876663208, 1549.249375879764557, 248.44852876663208, 1507.459169387817383, 248.44852876663208 ],
					"source" : [ "obj-329", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-104", 0 ],
					"order" : 1,
					"source" : [ "obj-33", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-105", 0 ],
					"order" : 3,
					"source" : [ "obj-33", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-106", 0 ],
					"order" : 5,
					"source" : [ "obj-33", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-107", 0 ],
					"order" : 8,
					"source" : [ "obj-33", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-95", 7 ],
					"midpoints" : [ 725.676456928253174, 150.163955688476562, 978.128753423690796, 150.163955688476562, 978.128753423690796, 132.163955688476562, 1095.128753423690796, 132.163955688476562, 1095.128753423690796, 66.163955688476562, 1085.941157341003418, 66.163955688476562 ],
					"order" : 0,
					"source" : [ "obj-33", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-96", 0 ],
					"order" : 4,
					"source" : [ "obj-33", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-97", 0 ],
					"order" : 6,
					"source" : [ "obj-33", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-98", 0 ],
					"order" : 7,
					"source" : [ "obj-33", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-99", 0 ],
					"order" : 2,
					"source" : [ "obj-33", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-333", 0 ],
					"midpoints" : [ 1507.459169387817383, 149.44852876663208, 1483.249375879764557, 149.44852876663208, 1483.249375879764557, 248.44852876663208, 1507.459169387817383, 248.44852876663208 ],
					"source" : [ "obj-330", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-345", 2 ],
					"source" : [ "obj-331", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-331", 1 ],
					"midpoints" : [ 1507.459169387817383, 356.823529243469238, 1531.255087852478027, 356.823529243469238 ],
					"source" : [ "obj-332", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-332", 0 ],
					"midpoints" : [ 1507.459169387817383, 287.44852876663208, 1507.459169387817383, 287.44852876663208 ],
					"source" : [ "obj-333", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-339", 2 ],
					"midpoints" : [ 1268.683661460876465, 323.823529243469238, 1269.193865776062012, 323.823529243469238 ],
					"source" : [ "obj-334", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-345", 1 ],
					"source" : [ "obj-335", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-335", 0 ],
					"midpoints" : [ 1225.826519012451172, 464.823529243469238, 1225.826519012451172, 464.823529243469238 ],
					"source" : [ "obj-336", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-336", 0 ],
					"midpoints" : [ 1225.826519012451172, 434.823529243469238, 1225.826519012451172, 434.823529243469238 ],
					"source" : [ "obj-337", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-341", 0 ],
					"midpoints" : [ 1225.826519012451172, 383.823529243469238, 1225.826519012451172, 383.823529243469238 ],
					"source" : [ "obj-338", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-338", 1 ],
					"midpoints" : [ 1244.193865776062012, 350.823529243469238, 1253.826519012451172, 350.823529243469238 ],
					"source" : [ "obj-339", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-338", 0 ],
					"midpoints" : [ 1225.826519012451172, 320.823529243469238, 1225.826519012451172, 320.823529243469238 ],
					"source" : [ "obj-340", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-339", 0 ],
					"midpoints" : [ 1236.326519012451172, 320.823529243469238, 1244.193865776062012, 320.823529243469238 ],
					"source" : [ "obj-340", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-337", 0 ],
					"midpoints" : [ 1225.826519012451172, 410.823529243469238, 1225.826519012451172, 410.823529243469238 ],
					"source" : [ "obj-341", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-340", 0 ],
					"midpoints" : [ 1225.826519012451172, 293.823529243469238, 1225.826519012451172, 293.823529243469238 ],
					"source" : [ "obj-342", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-268", 0 ],
					"order" : 1,
					"source" : [ "obj-343", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-313", 0 ],
					"order" : 0,
					"source" : [ "obj-343", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-345", 0 ],
					"midpoints" : [ 1227.867335319519043, 557.823529243469238, 1227.867335319519043, 557.823529243469238 ],
					"source" : [ "obj-344", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-343", 0 ],
					"midpoints" : [ 1227.867335319519043, 584.0031578540802, 1227.867335319519043, 584.0031578540802 ],
					"source" : [ "obj-345", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-429", 0 ],
					"source" : [ "obj-345", 2 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-303", 0 ],
					"midpoints" : [ 1211.540804862976074, 215.823529243469238, 1187.822325229644775, 215.823529243469238, 1187.822325229644775, 179.823529243469238, 1142.153050422668457, 179.823529243469238 ],
					"order" : 2,
					"source" : [ "obj-346", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-306", 0 ],
					"midpoints" : [ 1211.540804862976074, 215.823529243469238, 1304.822325229644775, 215.823529243469238, 1304.822325229644775, 185.823529243469238, 1319.704069137573242, 185.823529243469238 ],
					"order" : 0,
					"source" : [ "obj-346", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-347", 0 ],
					"midpoints" : [ 1211.540804862976074, 215.823529243469238, 1127.822325229644775, 215.823529243469238, 1127.822325229644775, 239.823529243469238, 1142.153050422668457, 239.823529243469238 ],
					"order" : 1,
					"source" : [ "obj-346", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-305", 0 ],
					"midpoints" : [ 1142.153050422668457, 275.823529243469238, 1211.822325229644775, 275.823529243469238, 1211.822325229644775, 266.823529243469238, 1292.822325229644775, 266.823529243469238, 1292.822325229644775, 221.823529243469238, 1315.6224365234375, 221.823529243469238 ],
					"source" : [ "obj-347", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-363", 0 ],
					"order" : 3,
					"source" : [ "obj-350", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-364", 0 ],
					"order" : 4,
					"source" : [ "obj-350", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-365", 0 ],
					"order" : 6,
					"source" : [ "obj-350", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-366", 0 ],
					"order" : 1,
					"source" : [ "obj-350", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-371", 0 ],
					"order" : 0,
					"source" : [ "obj-350", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-372", 0 ],
					"order" : 2,
					"source" : [ "obj-350", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-373", 0 ],
					"order" : 5,
					"source" : [ "obj-350", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-374", 0 ],
					"order" : 7,
					"source" : [ "obj-350", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-353", 0 ],
					"source" : [ "obj-351", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-368", 0 ],
					"midpoints" : [ 3133.5, 235.797657012939453, 3133.5, 235.797657012939453 ],
					"source" : [ "obj-354", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-375", 0 ],
					"midpoints" : [ 3249.5, 235.797657012939453, 3249.5, 235.797657012939453 ],
					"source" : [ "obj-355", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-369", 0 ],
					"midpoints" : [ 3077.5, 235.797657012939453, 3077.5, 235.797657012939453 ],
					"source" : [ "obj-356", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-367", 0 ],
					"midpoints" : [ 3189.5, 235.797657012939453, 3189.5, 235.797657012939453 ],
					"source" : [ "obj-357", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-370", 0 ],
					"source" : [ "obj-358", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-376", 0 ],
					"source" : [ "obj-359", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-377", 0 ],
					"source" : [ "obj-360", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-378", 0 ],
					"midpoints" : [ 3077.5, 124.987485408782959, 3073.5, 124.987485408782959 ],
					"source" : [ "obj-361", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-357", 0 ],
					"order" : 1,
					"source" : [ "obj-363", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-362", 6 ],
					"midpoints" : [ 3189.5, 205.987485408782959, 3219.481708765029907, 205.987485408782959, 3219.481708765029907, 163.987485408782959, 3300.481708765029907, 163.987485408782959, 3300.481708765029907, 112.987485408782959, 3376.5, 112.987485408782959 ],
					"order" : 0,
					"source" : [ "obj-363", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-354", 0 ],
					"order" : 1,
					"source" : [ "obj-364", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-362", 5 ],
					"midpoints" : [ 3133.5, 205.987485408782959, 3162.481708765029907, 205.987485408782959, 3162.481708765029907, 163.987485408782959, 3300.481708765029907, 163.987485408782959, 3300.481708765029907, 112.987485408782959, 3366.0, 112.987485408782959 ],
					"order" : 0,
					"source" : [ "obj-364", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-356", 0 ],
					"order" : 1,
					"source" : [ "obj-365", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-362", 4 ],
					"midpoints" : [ 3077.5, 205.987485408782959, 3054.481708765029907, 205.987485408782959, 3054.481708765029907, 70.987485408782959, 3355.5, 70.987485408782959 ],
					"order" : 0,
					"source" : [ "obj-365", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-358", 0 ],
					"midpoints" : [ 3249.5, 124.987485408782959, 3249.5, 124.987485408782959 ],
					"order" : 1,
					"source" : [ "obj-366", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-362", 3 ],
					"midpoints" : [ 3249.5, 124.987485408782959, 3291.481708765029907, 124.987485408782959, 3291.481708765029907, 112.987485408782959, 3345.0, 112.987485408782959 ],
					"order" : 0,
					"source" : [ "obj-366", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-380", 0 ],
					"midpoints" : [ 3189.5, 283.797657012939453, 3077.5, 283.797657012939453 ],
					"source" : [ "obj-367", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-380", 0 ],
					"midpoints" : [ 3133.5, 283.797657012939453, 3077.5, 283.797657012939453 ],
					"source" : [ "obj-368", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-380", 0 ],
					"midpoints" : [ 3077.5, 295.797657012939453, 3077.5, 295.797657012939453 ],
					"source" : [ "obj-369", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-380", 0 ],
					"midpoints" : [ 3249.5, 154.987485408782959, 3237.481708765029907, 154.987485408782959, 3237.481708765029907, 253.987485408782959, 3077.5, 253.987485408782959 ],
					"source" : [ "obj-370", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-355", 0 ],
					"order" : 1,
					"source" : [ "obj-371", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-362", 7 ],
					"midpoints" : [ 3249.5, 205.987485408782959, 3279.481708765029907, 205.987485408782959, 3279.481708765029907, 187.987485408782959, 3396.481708765029907, 187.987485408782959, 3396.481708765029907, 121.987485408782959, 3387.0, 121.987485408782959 ],
					"order" : 0,
					"source" : [ "obj-371", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-359", 0 ],
					"midpoints" : [ 3193.5, 124.987485408782959, 3193.5, 124.987485408782959 ],
					"order" : 1,
					"source" : [ "obj-372", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-362", 2 ],
					"midpoints" : [ 3193.5, 124.987485408782959, 3291.481708765029907, 124.987485408782959, 3291.481708765029907, 112.987485408782959, 3334.5, 112.987485408782959 ],
					"order" : 0,
					"source" : [ "obj-372", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-360", 0 ],
					"order" : 1,
					"source" : [ "obj-373", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-362", 1 ],
					"order" : 0,
					"source" : [ "obj-373", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-361", 0 ],
					"order" : 1,
					"source" : [ "obj-374", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-362", 0 ],
					"midpoints" : [ 3077.5, 124.987485408782959, 3111.481708765029907, 124.987485408782959, 3111.481708765029907, 70.987485408782959, 3313.5, 70.987485408782959 ],
					"order" : 0,
					"source" : [ "obj-374", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-380", 0 ],
					"midpoints" : [ 3249.5, 283.797657012939453, 3077.5, 283.797657012939453 ],
					"source" : [ "obj-375", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-380", 0 ],
					"midpoints" : [ 3189.5, 154.987485408782959, 3180.481708765029907, 154.987485408782959, 3180.481708765029907, 253.987485408782959, 3077.5, 253.987485408782959 ],
					"source" : [ "obj-376", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-380", 0 ],
					"midpoints" : [ 3133.5, 154.987485408782959, 3120.481708765029907, 154.987485408782959, 3120.481708765029907, 253.987485408782959, 3077.5, 253.987485408782959 ],
					"source" : [ "obj-377", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-380", 0 ],
					"midpoints" : [ 3073.5, 154.987485408782959, 3054.481708765029907, 154.987485408782959, 3054.481708765029907, 253.987485408782959, 3077.5, 253.987485408782959 ],
					"source" : [ "obj-378", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-409", 0 ],
					"midpoints" : [ 3073.5, 709.823529720306396, 2703.352955341339111, 709.823529720306396, 2703.352955341339111, 571.823529720306396, 2665.5, 571.823529720306396 ],
					"source" : [ "obj-379", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-257", 0 ],
					"midpoints" : [ 771.404754638671875, 1831.324594855308533, 771.404754638671875, 1831.324594855308533 ],
					"source" : [ "obj-38", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-379", 0 ],
					"midpoints" : [ 3077.5, 337.591301202774048, 3073.5, 337.591301202774048 ],
					"source" : [ "obj-380", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-382", 0 ],
					"midpoints" : [ 2309.5, 262.823529720306396, 2309.5, 262.823529720306396 ],
					"source" : [ "obj-381", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-383", 0 ],
					"midpoints" : [ 2309.5, 289.823529720306396, 2298.352955341339111, 289.823529720306396, 2298.352955341339111, 262.823529720306396, 2481.352955341339111, 262.823529720306396, 2481.352955341339111, 268.823529720306396, 2485.5, 268.823529720306396 ],
					"source" : [ "obj-382", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-384", 2 ],
					"midpoints" : [ 2485.5, 298.823529720306396, 2532.352955341339111, 298.823529720306396, 2532.352955341339111, 232.823529720306396, 2521.5, 232.823529720306396 ],
					"source" : [ "obj-383", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-392", 0 ],
					"midpoints" : [ 2489.5, 262.823529720306396, 2358.352955341339111, 262.823529720306396, 2358.352955341339111, 76.823529720306396, 2385.5, 76.823529720306396 ],
					"source" : [ "obj-384", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-420", 0 ],
					"midpoints" : [ 2521.5, 322.823529720306396, 2412.352955341339111, 322.823529720306396, 2412.352955341339111, 313.823529720306396, 2393.5, 313.823529720306396 ],
					"source" : [ "obj-384", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-393", 0 ],
					"midpoints" : [ 2730.642857142857338, 112.448529243469238, 2770.780005991458893, 112.448529243469238, 2770.780005991458893, 169.448529243469238, 2779.780005991458893, 169.448529243469238, 2779.780005991458893, 202.448529243469238, 2789.5, 202.448529243469238 ],
					"source" : [ "obj-386", 6 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-394", 0 ],
					"midpoints" : [ 2719.785714285714221, 112.448529243469238, 2710.780005991458893, 112.448529243469238, 2710.780005991458893, 169.448529243469238, 2722.780005991458893, 169.448529243469238, 2722.780005991458893, 202.448529243469238, 2733.5, 202.448529243469238 ],
					"source" : [ "obj-386", 5 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-395", 0 ],
					"midpoints" : [ 2708.928571428571558, 112.448529243469238, 2653.780005991458893, 112.448529243469238, 2653.780005991458893, 205.448529243469238, 2677.5, 205.448529243469238 ],
					"source" : [ "obj-386", 4 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-396", 0 ],
					"midpoints" : [ 2698.071428571428442, 112.448529243469238, 2849.5, 112.448529243469238 ],
					"source" : [ "obj-386", 3 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-401", 0 ],
					"midpoints" : [ 2741.5, 112.448529243469238, 2827.780005991458893, 112.448529243469238, 2827.780005991458893, 169.448529243469238, 2836.780005991458893, 169.448529243469238, 2836.780005991458893, 202.448529243469238, 2849.5, 202.448529243469238 ],
					"source" : [ "obj-386", 7 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-402", 0 ],
					"midpoints" : [ 2687.214285714285779, 112.448529243469238, 2793.5, 112.448529243469238 ],
					"source" : [ "obj-386", 2 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-403", 0 ],
					"midpoints" : [ 2676.357142857142662, 112.448529243469238, 2733.5, 112.448529243469238 ],
					"source" : [ "obj-386", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-404", 0 ],
					"midpoints" : [ 2665.5, 112.448529243469238, 2677.5, 112.448529243469238 ],
					"source" : [ "obj-386", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-387", 0 ],
					"midpoints" : [ 2909.5, 193.448529243469238, 2909.5, 193.448529243469238 ],
					"source" : [ "obj-388", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-393", 0 ],
					"midpoints" : [ 2597.5, 205.448529243469238, 2789.5, 205.448529243469238 ],
					"order" : 3,
					"source" : [ "obj-389", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-394", 0 ],
					"midpoints" : [ 2597.5, 205.448529243469238, 2733.5, 205.448529243469238 ],
					"order" : 4,
					"source" : [ "obj-389", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-395", 0 ],
					"midpoints" : [ 2597.5, 205.448529243469238, 2677.5, 205.448529243469238 ],
					"order" : 6,
					"source" : [ "obj-389", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-396", 0 ],
					"midpoints" : [ 2597.5, 121.448529243469238, 2710.780005991458893, 121.448529243469238, 2710.780005991458893, 112.448529243469238, 2849.5, 112.448529243469238 ],
					"order" : 1,
					"source" : [ "obj-389", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-401", 0 ],
					"midpoints" : [ 2597.5, 121.448529243469238, 2710.780005991458893, 121.448529243469238, 2710.780005991458893, 112.448529243469238, 2827.780005991458893, 112.448529243469238, 2827.780005991458893, 169.448529243469238, 2836.780005991458893, 169.448529243469238, 2836.780005991458893, 202.448529243469238, 2849.5, 202.448529243469238 ],
					"order" : 0,
					"source" : [ "obj-389", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-402", 0 ],
					"midpoints" : [ 2597.5, 121.448529243469238, 2710.780005991458893, 121.448529243469238, 2710.780005991458893, 112.448529243469238, 2793.5, 112.448529243469238 ],
					"order" : 2,
					"source" : [ "obj-389", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-403", 0 ],
					"midpoints" : [ 2597.5, 124.448529243469238, 2733.5, 124.448529243469238 ],
					"order" : 5,
					"source" : [ "obj-389", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-404", 0 ],
					"midpoints" : [ 2597.5, 124.448529243469238, 2677.5, 124.448529243469238 ],
					"order" : 7,
					"source" : [ "obj-389", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-384", 1 ],
					"midpoints" : [ 2385.5, 229.823529720306396, 2505.5, 229.823529720306396 ],
					"source" : [ "obj-392", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-388", 6 ],
					"midpoints" : [ 2789.5, 247.448529243469238, 2818.780005991458893, 247.448529243469238, 2818.780005991458893, 205.448529243469238, 2899.780005991458893, 205.448529243469238, 2899.780005991458893, 154.448529243469238, 2972.5, 154.448529243469238 ],
					"order" : 0,
					"source" : [ "obj-393", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-397", 0 ],
					"midpoints" : [ 2789.5, 247.448529243469238, 2789.5, 247.448529243469238 ],
					"order" : 1,
					"source" : [ "obj-393", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-388", 5 ],
					"midpoints" : [ 2733.5, 247.448529243469238, 2761.780005991458893, 247.448529243469238, 2761.780005991458893, 205.448529243469238, 2899.780005991458893, 205.448529243469238, 2899.780005991458893, 154.448529243469238, 2962.0, 154.448529243469238 ],
					"order" : 0,
					"source" : [ "obj-394", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-398", 0 ],
					"midpoints" : [ 2733.5, 247.448529243469238, 2733.5, 247.448529243469238 ],
					"order" : 1,
					"source" : [ "obj-394", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-388", 4 ],
					"midpoints" : [ 2677.5, 247.448529243469238, 2653.780005991458893, 247.448529243469238, 2653.780005991458893, 112.448529243469238, 2951.5, 112.448529243469238 ],
					"order" : 0,
					"source" : [ "obj-395", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-399", 0 ],
					"midpoints" : [ 2677.5, 247.448529243469238, 2677.5, 247.448529243469238 ],
					"order" : 1,
					"source" : [ "obj-395", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-388", 3 ],
					"midpoints" : [ 2849.5, 166.448529243469238, 2890.780005991458893, 166.448529243469238, 2890.780005991458893, 154.448529243469238, 2941.0, 154.448529243469238 ],
					"order" : 0,
					"source" : [ "obj-396", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-400", 0 ],
					"midpoints" : [ 2849.5, 166.448529243469238, 2849.5, 166.448529243469238 ],
					"order" : 1,
					"source" : [ "obj-396", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-411", 0 ],
					"midpoints" : [ 2789.5, 295.448529243469238, 2677.5, 295.448529243469238 ],
					"source" : [ "obj-397", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-411", 0 ],
					"midpoints" : [ 2733.5, 295.448529243469238, 2677.5, 295.448529243469238 ],
					"source" : [ "obj-398", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-411", 0 ],
					"midpoints" : [ 2677.5, 307.448529243469238, 2677.5, 307.448529243469238 ],
					"source" : [ "obj-399", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-1", 0 ],
					"midpoints" : [ 97.255101203918457, 549.0, 97.255101203918457, 549.0 ],
					"source" : [ "obj-4", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-411", 0 ],
					"midpoints" : [ 2849.5, 196.448529243469238, 2836.780005991458893, 196.448529243469238, 2836.780005991458893, 295.448529243469238, 2677.5, 295.448529243469238 ],
					"source" : [ "obj-400", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-388", 7 ],
					"midpoints" : [ 2849.5, 247.448529243469238, 2878.780005991458893, 247.448529243469238, 2878.780005991458893, 229.448529243469238, 2995.780005991458893, 229.448529243469238, 2995.780005991458893, 163.448529243469238, 2983.0, 163.448529243469238 ],
					"order" : 0,
					"source" : [ "obj-401", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-405", 0 ],
					"midpoints" : [ 2849.5, 247.448529243469238, 2849.5, 247.448529243469238 ],
					"order" : 1,
					"source" : [ "obj-401", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-388", 2 ],
					"midpoints" : [ 2793.5, 166.448529243469238, 2890.780005991458893, 166.448529243469238, 2890.780005991458893, 154.448529243469238, 2930.5, 154.448529243469238 ],
					"order" : 0,
					"source" : [ "obj-402", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-406", 0 ],
					"midpoints" : [ 2793.5, 166.448529243469238, 2793.5, 166.448529243469238 ],
					"order" : 1,
					"source" : [ "obj-402", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-388", 1 ],
					"midpoints" : [ 2733.5, 166.448529243469238, 2770.780005991458893, 166.448529243469238, 2770.780005991458893, 112.448529243469238, 2920.0, 112.448529243469238 ],
					"order" : 0,
					"source" : [ "obj-403", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-407", 0 ],
					"midpoints" : [ 2733.5, 166.448529243469238, 2733.5, 166.448529243469238 ],
					"order" : 1,
					"source" : [ "obj-403", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-388", 0 ],
					"midpoints" : [ 2677.5, 166.448529243469238, 2710.780005991458893, 166.448529243469238, 2710.780005991458893, 112.448529243469238, 2909.5, 112.448529243469238 ],
					"order" : 0,
					"source" : [ "obj-404", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-408", 0 ],
					"midpoints" : [ 2677.5, 166.448529243469238, 2677.5, 166.448529243469238 ],
					"order" : 1,
					"source" : [ "obj-404", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-411", 0 ],
					"midpoints" : [ 2849.5, 295.448529243469238, 2677.5, 295.448529243469238 ],
					"source" : [ "obj-405", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-411", 0 ],
					"midpoints" : [ 2793.5, 196.448529243469238, 2779.780005991458893, 196.448529243469238, 2779.780005991458893, 295.448529243469238, 2677.5, 295.448529243469238 ],
					"source" : [ "obj-406", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-411", 0 ],
					"midpoints" : [ 2733.5, 196.448529243469238, 2719.780005991458893, 196.448529243469238, 2719.780005991458893, 295.448529243469238, 2677.5, 295.448529243469238 ],
					"source" : [ "obj-407", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-411", 0 ],
					"midpoints" : [ 2677.5, 196.448529243469238, 2653.780005991458893, 196.448529243469238, 2653.780005991458893, 295.448529243469238, 2677.5, 295.448529243469238 ],
					"source" : [ "obj-408", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-423", 2 ],
					"source" : [ "obj-409", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-409", 1 ],
					"midpoints" : [ 2677.5, 403.823529720306396, 2699.5, 403.823529720306396 ],
					"source" : [ "obj-410", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-410", 0 ],
					"midpoints" : [ 2677.5, 334.448529243469238, 2677.5, 334.448529243469238 ],
					"source" : [ "obj-411", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-417", 2 ],
					"midpoints" : [ 2437.5, 370.823529720306396, 2438.5, 370.823529720306396 ],
					"source" : [ "obj-412", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-423", 1 ],
					"source" : [ "obj-413", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-413", 0 ],
					"midpoints" : [ 2393.5, 511.823529720306396, 2393.5, 511.823529720306396 ],
					"source" : [ "obj-414", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-414", 0 ],
					"midpoints" : [ 2393.5, 481.823529720306396, 2393.5, 481.823529720306396 ],
					"source" : [ "obj-415", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-419", 0 ],
					"midpoints" : [ 2393.5, 430.823529720306396, 2393.5, 430.823529720306396 ],
					"source" : [ "obj-416", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-416", 1 ],
					"midpoints" : [ 2413.5, 397.823529720306396, 2421.5, 397.823529720306396 ],
					"source" : [ "obj-417", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-416", 0 ],
					"midpoints" : [ 2393.5, 367.823529720306396, 2393.5, 367.823529720306396 ],
					"source" : [ "obj-418", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-417", 0 ],
					"midpoints" : [ 2404.0, 367.823529720306396, 2413.5, 367.823529720306396 ],
					"source" : [ "obj-418", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-415", 0 ],
					"midpoints" : [ 2393.5, 457.823529720306396, 2393.5, 457.823529720306396 ],
					"source" : [ "obj-419", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-250", 0 ],
					"midpoints" : [ 766.642849922180176, 1975.324594855308533, 766.642849922180176, 1975.324594855308533 ],
					"source" : [ "obj-42", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-418", 0 ],
					"midpoints" : [ 2393.5, 340.823529720306396, 2393.5, 340.823529720306396 ],
					"source" : [ "obj-420", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-351", 0 ],
					"order" : 1,
					"source" : [ "obj-421", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-391", 0 ],
					"order" : 0,
					"source" : [ "obj-421", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-423", 0 ],
					"midpoints" : [ 2397.5, 604.823529720306396, 2397.5, 604.823529720306396 ],
					"source" : [ "obj-422", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-421", 0 ],
					"midpoints" : [ 2397.5, 631.003158330917358, 2397.5, 631.003158330917358 ],
					"source" : [ "obj-423", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-429", 0 ],
					"source" : [ "obj-423", 2 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-381", 0 ],
					"midpoints" : [ 2381.5, 262.823529720306396, 2358.352955341339111, 262.823529720306396, 2358.352955341339111, 226.823529720306396, 2309.5, 226.823529720306396 ],
					"order" : 2,
					"source" : [ "obj-424", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-384", 0 ],
					"midpoints" : [ 2381.5, 262.823529720306396, 2475.352955341339111, 262.823529720306396, 2475.352955341339111, 232.823529720306396, 2489.5, 232.823529720306396 ],
					"order" : 0,
					"source" : [ "obj-424", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-425", 0 ],
					"midpoints" : [ 2381.5, 262.823529720306396, 2298.352955341339111, 262.823529720306396, 2298.352955341339111, 286.823529720306396, 2309.5, 286.823529720306396 ],
					"order" : 1,
					"source" : [ "obj-424", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-383", 0 ],
					"midpoints" : [ 2309.5, 322.823529720306396, 2382.352955341339111, 322.823529720306396, 2382.352955341339111, 313.823529720306396, 2463.352955341339111, 313.823529720306396, 2463.352955341339111, 268.823529720306396, 2485.5, 268.823529720306396 ],
					"source" : [ "obj-425", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-431", 1 ],
					"order" : 0,
					"source" : [ "obj-429", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-431", 0 ],
					"order" : 1,
					"source" : [ "obj-429", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-2", 7 ],
					"midpoints" : [ 440.614550650119781, 56.624999523162842, 526.427050650119781, 56.624999523162842, 526.427050650119781, 113.624999523162842, 535.427050650119781, 113.624999523162842, 535.427050650119781, 146.624999523162842, 549.908664762973785, 146.624999523162842, 549.908664762973785, 191.624999523162842, 577.427050650119781, 191.624999523162842, 577.427050650119781, 173.624999523162842, 694.427050650119781, 173.624999523162842, 694.427050650119781, 107.624999523162842, 685.173369467258453, 107.624999523162842 ],
					"order" : 0,
					"source" : [ "obj-43", 7 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-222", 0 ],
					"order" : 1,
					"source" : [ "obj-43", 7 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-224", 0 ],
					"midpoints" : [ 364.614550650119781, 56.624999523162842, 376.379256308078766, 56.624999523162842 ],
					"source" : [ "obj-43", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-225", 0 ],
					"midpoints" : [ 375.471693507262614, 56.624999523162842, 435.202784597873688, 56.624999523162842 ],
					"source" : [ "obj-43", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-226", 0 ],
					"midpoints" : [ 386.328836364405504, 56.624999523162842, 492.555724680423737, 56.624999523162842 ],
					"source" : [ "obj-43", 2 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-228", 0 ],
					"midpoints" : [ 429.757407792976892, 56.624999523162842, 469.427050650119781, 56.624999523162842, 469.427050650119781, 113.624999523162842, 478.427050650119781, 113.624999523162842, 478.427050650119781, 146.624999523162842, 491.085136473178864, 146.624999523162842 ],
					"source" : [ "obj-43", 6 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-229", 0 ],
					"midpoints" : [ 418.900264935834059, 56.624999523162842, 409.427050650119781, 56.624999523162842, 409.427050650119781, 113.624999523162842, 421.427050650119781, 113.624999523162842, 421.427050650119781, 146.624999523162842, 433.732196390628815, 146.624999523162842 ],
					"source" : [ "obj-43", 5 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-230", 0 ],
					"midpoints" : [ 408.043122078691226, 56.624999523162842, 352.427050650119781, 56.624999523162842, 352.427050650119781, 149.624999523162842, 376.379256308078766, 149.624999523162842 ],
					"source" : [ "obj-43", 4 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-231", 0 ],
					"midpoints" : [ 397.185979221548337, 56.624999523162842, 549.908664762973785, 56.624999523162842 ],
					"source" : [ "obj-43", 3 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-137", 0 ],
					"midpoints" : [ 1190.452369689941406, 1816.324594855308533, 1190.452369689941406, 1816.324594855308533 ],
					"source" : [ "obj-45", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-57", 1 ],
					"midpoints" : [ 376.379256308078766, 348.0, 363.5, 348.0 ],
					"source" : [ "obj-46", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-46", 0 ],
					"midpoints" : [ 376.379256308078766, 278.624999523162842, 376.379256308078766, 278.624999523162842 ],
					"source" : [ "obj-47", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-51", 1 ],
					"source" : [ "obj-49", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-1", 2 ],
					"source" : [ "obj-57", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-78", 0 ],
					"midpoints" : [ 81.125, 207.0, 174.0, 207.0, 174.0, 177.0, 189.1875, 177.0 ],
					"order" : 0,
					"source" : [ "obj-62", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-80", 0 ],
					"midpoints" : [ 81.125, 207.0, -3.0, 207.0, -3.0, 231.0, 10.972533702850342, 231.0 ],
					"order" : 1,
					"source" : [ "obj-62", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-87", 0 ],
					"midpoints" : [ 81.125, 207.0, 57.0, 207.0, 57.0, 171.0, 10.972533702850342, 171.0 ],
					"order" : 2,
					"source" : [ "obj-62", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-241", 0 ],
					"order" : 0,
					"source" : [ "obj-7", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-28", 0 ],
					"order" : 1,
					"source" : [ "obj-7", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-11", 0 ],
					"midpoints" : [ 221.1875, 267.0, 111.0, 267.0, 111.0, 258.0, 94.115388751029968, 258.0 ],
					"source" : [ "obj-78", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-9", 0 ],
					"midpoints" : [ 189.1875, 207.0, 57.0, 207.0, 57.0, 21.0, 87.625, 21.0 ],
					"source" : [ "obj-78", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-78", 2 ],
					"midpoints" : [ 184.5, 243.0, 231.0, 243.0, 231.0, 177.0, 221.1875, 177.0 ],
					"source" : [ "obj-79", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-253", 0 ],
					"midpoints" : [ 771.404754638671875, 1741.324594855308533, 771.404754638671875, 1741.324594855308533 ],
					"source" : [ "obj-8", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-79", 0 ],
					"midpoints" : [ 10.972533702850342, 267.0, 81.0, 267.0, 81.0, 258.0, 162.0, 258.0, 162.0, 213.0, 184.5, 213.0 ],
					"source" : [ "obj-80", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-79", 0 ],
					"midpoints" : [ 10.972533702850342, 234.0, -3.0, 234.0, -3.0, 207.0, 180.0, 207.0, 180.0, 213.0, 184.5, 213.0 ],
					"source" : [ "obj-85", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-85", 0 ],
					"midpoints" : [ 10.972533702850342, 207.0, 10.972533702850342, 207.0 ],
					"source" : [ "obj-87", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-44", 0 ],
					"midpoints" : [ 87.625, 174.0, 245.4375, 174.0 ],
					"order" : 0,
					"source" : [ "obj-9", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-78", 1 ],
					"midpoints" : [ 87.625, 174.0, 205.1875, 174.0 ],
					"order" : 1,
					"source" : [ "obj-9", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-140", 0 ],
					"order" : 1,
					"source" : [ "obj-96", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-95", 6 ],
					"midpoints" : [ 891.852924346923828, 150.163955688476562, 918.128753423690796, 150.163955688476562, 918.128753423690796, 108.163955688476562, 999.128753423690796, 108.163955688476562, 999.128753423690796, 57.163955688476562, 1075.441157341003418, 57.163955688476562 ],
					"order" : 0,
					"source" : [ "obj-96", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-145", 0 ],
					"order" : 1,
					"source" : [ "obj-97", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-95", 5 ],
					"midpoints" : [ 833.029396057128906, 150.163955688476562, 861.128753423690796, 150.163955688476562, 861.128753423690796, 108.163955688476562, 999.128753423690796, 108.163955688476562, 999.128753423690796, 57.163955688476562, 1064.941157341003418, 57.163955688476562 ],
					"order" : 0,
					"source" : [ "obj-97", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-141", 0 ],
					"order" : 1,
					"source" : [ "obj-98", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-95", 4 ],
					"midpoints" : [ 777.14704418182373, 150.163955688476562, 753.128753423690796, 150.163955688476562, 753.128753423690796, 15.163955688476562, 1054.441157341003418, 15.163955688476562 ],
					"order" : 0,
					"source" : [ "obj-98", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-135", 0 ],
					"midpoints" : [ 950.67645263671875, 69.163955688476562, 950.67645263671875, 69.163955688476562 ],
					"order" : 1,
					"source" : [ "obj-99", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-95", 3 ],
					"midpoints" : [ 950.67645263671875, 69.163955688476562, 990.128753423690796, 69.163955688476562, 990.128753423690796, 57.163955688476562, 1043.941157341003418, 57.163955688476562 ],
					"order" : 0,
					"source" : [ "obj-99", 0 ]
				}

			}
 ],
		"parameters" : 		{
			"obj-104" : [ "-[12]", "1", 0 ],
			"obj-105" : [ "-[13]", "1", 0 ],
			"obj-106" : [ "-[14]", "1", 0 ],
			"obj-107" : [ "-[15]", "1", 0 ],
			"obj-126" : [ "button[4]", "button[4]", 0 ],
			"obj-13" : [ "button[3]", "button[3]", 0 ],
			"obj-15" : [ "button[7]", "button[7]", 0 ],
			"obj-168" : [ "button", "button", 0 ],
			"obj-180" : [ "button[5]", "button[5]", 0 ],
			"obj-192" : [ "button[6]", "button[6]", 0 ],
			"obj-219" : [ "number", "number", 0 ],
			"obj-224" : [ "-", "1", 0 ],
			"obj-225" : [ "-[1]", "1", 0 ],
			"obj-226" : [ "-[2]", "1", 0 ],
			"obj-227" : [ "-[3]", "1", 0 ],
			"obj-228" : [ "-[4]", "1", 0 ],
			"obj-229" : [ "-[5]", "1", 0 ],
			"obj-230" : [ "-[6]", "1", 0 ],
			"obj-231" : [ "-[7]", "1", 0 ],
			"obj-237" : [ "live.text", "live.text", 0 ],
			"obj-285" : [ "-[23]", "1", 0 ],
			"obj-286" : [ "-[29]", "1", 0 ],
			"obj-287" : [ "-[31]", "1", 0 ],
			"obj-288" : [ "-[32]", "1", 0 ],
			"obj-293" : [ "-[33]", "1", 0 ],
			"obj-294" : [ "-[24]", "1", 0 ],
			"obj-295" : [ "-[30]", "1", 0 ],
			"obj-296" : [ "-[34]", "1", 0 ],
			"obj-305" : [ "live.numbox[1]", "live.numbox", 0 ],
			"obj-314" : [ "multislider[1]", "multislider", 0 ],
			"obj-315" : [ "-[35]", "1", 0 ],
			"obj-316" : [ "-[25]", "1", 0 ],
			"obj-317" : [ "-[36]", "1", 0 ],
			"obj-318" : [ "-[37]", "1", 0 ],
			"obj-323" : [ "-[26]", "1", 0 ],
			"obj-324" : [ "-[38]", "1", 0 ],
			"obj-325" : [ "-[39]", "1", 0 ],
			"obj-326" : [ "-[27]", "1", 0 ],
			"obj-334" : [ "button[1]", "button", 0 ],
			"obj-344" : [ "live.dial[1]", "live.dial", 0 ],
			"obj-346" : [ "live.menu[1]", "live.menu", 0 ],
			"obj-363" : [ "-[40]", "1", 0 ],
			"obj-364" : [ "-[41]", "1", 0 ],
			"obj-365" : [ "-[28]", "1", 0 ],
			"obj-366" : [ "-[42]", "1", 0 ],
			"obj-371" : [ "-[43]", "1", 0 ],
			"obj-372" : [ "-[44]", "1", 0 ],
			"obj-373" : [ "-[45]", "1", 0 ],
			"obj-374" : [ "-[46]", "1", 0 ],
			"obj-383" : [ "live.numbox[2]", "live.numbox", 0 ],
			"obj-392" : [ "multislider[2]", "multislider", 0 ],
			"obj-393" : [ "-[47]", "1", 0 ],
			"obj-394" : [ "-[48]", "1", 0 ],
			"obj-395" : [ "-[49]", "1", 0 ],
			"obj-396" : [ "-[50]", "1", 0 ],
			"obj-4" : [ "live.dial", "live.dial", 0 ],
			"obj-401" : [ "-[51]", "1", 0 ],
			"obj-402" : [ "-[52]", "1", 0 ],
			"obj-403" : [ "-[53]", "1", 0 ],
			"obj-404" : [ "-[54]", "1", 0 ],
			"obj-412" : [ "button[2]", "button", 0 ],
			"obj-422" : [ "live.dial[2]", "live.dial", 0 ],
			"obj-424" : [ "live.menu[2]", "live.menu", 0 ],
			"obj-431" : [ "live.gain~", "live.gain~", 0 ],
			"obj-49" : [ "umenu", "umenu", 0 ],
			"obj-62" : [ "live.menu", "live.menu", 0 ],
			"obj-79" : [ "live.numbox", "live.numbox", 0 ],
			"obj-9" : [ "multislider", "multislider", 0 ],
			"obj-96" : [ "-[8]", "1", 0 ],
			"obj-97" : [ "-[9]", "1", 0 ],
			"obj-98" : [ "-[10]", "1", 0 ],
			"obj-99" : [ "-[11]", "1", 0 ],
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
		"parameter_map" : 		{
			"key" : 			{
				"live.button[2]" : 				{
					"srcname" : "0.modifiers.0.code.key",
					"min" : 0.0,
					"max" : 1.0,
					"flags" : 1,
					"trigger" : 1
				}
,
				"live.button[3]" : 				{
					"srcname" : "0.modifiers.1.code.key",
					"min" : 0.0,
					"max" : 1.0,
					"flags" : 1,
					"trigger" : 1
				}
,
				"live.button[4]" : 				{
					"srcname" : "0.modifiers.2.code.key",
					"min" : 0.0,
					"max" : 1.0,
					"flags" : 1,
					"trigger" : 1
				}

			}

		}
,
		"dependency_cache" : [ 			{
				"name" : "fluid.buf2list.mxo",
				"type" : "iLaX"
			}
, 			{
				"name" : "fluid.dataset~.mxo",
				"type" : "iLaX"
			}
, 			{
				"name" : "fluid.list2buf.mxo",
				"type" : "iLaX"
			}
, 			{
				"name" : "fluid.mlpregressor~.mxo",
				"type" : "iLaX"
			}
 ],
		"autosave" : 0,
		"styles" : [ 			{
				"name" : "rnbodefault",
				"default" : 				{
					"accentcolor" : [ 0.343034118413925, 0.506230533123016, 0.86220508813858, 1.0 ],
					"bgcolor" : [ 0.031372549019608, 0.125490196078431, 0.211764705882353, 1.0 ],
					"bgfillcolor" : 					{
						"angle" : 270.0,
						"autogradient" : 0.0,
						"color" : [ 0.031372549019608, 0.125490196078431, 0.211764705882353, 1.0 ],
						"color1" : [ 0.031372549019608, 0.125490196078431, 0.211764705882353, 1.0 ],
						"color2" : [ 0.263682, 0.004541, 0.038797, 1.0 ],
						"proportion" : 0.39,
						"type" : "color"
					}
,
					"color" : [ 0.929412, 0.929412, 0.352941, 1.0 ],
					"elementcolor" : [ 0.357540726661682, 0.515565991401672, 0.861786782741547, 1.0 ],
					"fontname" : [ "Lato" ],
					"fontsize" : [ 12.0 ],
					"stripecolor" : [ 0.258338063955307, 0.352425158023834, 0.511919498443604, 1.0 ],
					"textcolor_inverse" : [ 0.968627, 0.968627, 0.968627, 1 ]
				}
,
				"parentstyle" : "",
				"multi" : 0
			}
, 			{
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
		"oscsendudpaddr" : "localhost"
	}

}
