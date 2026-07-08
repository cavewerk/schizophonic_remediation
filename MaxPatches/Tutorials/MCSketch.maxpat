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
		"rect" : [ 567.0, 76.0, 353.0, 721.0 ],
		"gridsize" : [ 15.0, 15.0 ],
		"boxes" : [ 			{
				"box" : 				{
					"id" : "obj-23",
					"lastchannelcount" : 0,
					"maxclass" : "live.gain~",
					"numinlets" : 2,
					"numoutlets" : 5,
					"outlettype" : [ "signal", "signal", "", "float", "list" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1118.055608868598938, 240.277789235115051, 48.0, 136.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 1566.666741371154785, 1351.388953328132629, 48.0, 136.0 ],
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
					"id" : "obj-21",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 0,
					"patching_rect" : [ 1104.420000000000073, 393.379999999999995, 55.0, 22.0 ],
					"text" : "dac~ 1 2"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-6",
					"maxclass" : "live.grid",
					"numinlets" : 2,
					"numoutlets" : 6,
					"outlettype" : [ "", "", "", "", "", "" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1481.089999999999918, 636.720000000000027, 300.0, 150.0 ],
					"saved_attribute_attributes" : 					{
						"valueof" : 						{
							"parameter_invisible" : 1,
							"parameter_longname" : "live.grid",
							"parameter_modmode" : 0,
							"parameter_shortname" : "live.grid",
							"parameter_type" : 3
						}

					}
,
					"varname" : "live.grid"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-140",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 1628.749844670295715, 1048.749899983406067, 103.0, 22.0 ],
					"text" : "s soprano_seqOn"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-139",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 1513.749855637550354, 1049.999899864196777, 79.0, 22.0 ],
					"text" : "s alto_seqOn"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-138",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 1399.999866485595703, 1049.999899864196777, 87.0, 22.0 ],
					"text" : "s tenor_seqOn"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-137",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 1284.999877452850342, 1048.749899983406067, 85.0, 22.0 ],
					"text" : "s bass_seqOn"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-130",
					"linecount" : 8,
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 4,
					"outlettype" : [ "int", "", "", "" ],
					"patching_rect" : [ 1287.499877214431763, 927.499911546707153, 103.200001537799835, 116.0 ],
					"text" : "jk.push @button track1below @oncolor c @offcolor bl @textoff track1below @texton bam @mode t",
					"varname" : "jkpush2[1]"
				}

			}
, 			{
				"box" : 				{
					"activebgcolor" : [ 1.0, 0.5, 0.035, 1.0 ],
					"activebgoncolor" : [ 0.0, 1.0, 1.0, 1.0 ],
					"activetextcolor" : [ 0.15, 0.15, 0.15, 1.0 ],
					"bordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
					"focusbordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
					"fontsize" : 9.0,
					"id" : "obj-131",
					"maxclass" : "live.text",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1284.999877452850342, 909.999913215637207, 44.0, 15.0 ],
					"saved_attribute_attributes" : 					{
						"activebgcolor" : 						{
							"expression" : ""
						}
,
						"activebgoncolor" : 						{
							"expression" : ""
						}
,
						"activetextcolor" : 						{
							"expression" : ""
						}
,
						"activetextoncolor" : 						{
							"expression" : ""
						}
,
						"bordercolor" : 						{
							"expression" : ""
						}
,
						"focusbordercolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_enum" : [ "val1", "val2" ],
							"parameter_longname" : "pushtext2[1]",
							"parameter_mmax" : 1,
							"parameter_modmode" : 0,
							"parameter_shortname" : "pushtext2",
							"parameter_type" : 2
						}

					}
,
					"text" : "track1below",
					"texton" : "bam",
					"varname" : "pushtext2[1]"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-132",
					"linecount" : 8,
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 4,
					"outlettype" : [ "int", "", "", "" ],
					"patching_rect" : [ 1399.999866485595703, 927.499911546707153, 103.200001537799835, 116.0 ],
					"text" : "jk.push @button track2below @oncolor c @offcolor bl @textoff track2below @texton bam @mode t",
					"varname" : "jkpush3[1]"
				}

			}
, 			{
				"box" : 				{
					"activebgcolor" : [ 1.0, 0.5, 0.035, 1.0 ],
					"activebgoncolor" : [ 0.0, 1.0, 1.0, 1.0 ],
					"activetextcolor" : [ 0.15, 0.15, 0.15, 1.0 ],
					"bordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
					"focusbordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
					"fontsize" : 9.0,
					"id" : "obj-133",
					"maxclass" : "live.text",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1399.999866485595703, 911.249913096427917, 44.0, 15.0 ],
					"saved_attribute_attributes" : 					{
						"activebgcolor" : 						{
							"expression" : ""
						}
,
						"activebgoncolor" : 						{
							"expression" : ""
						}
,
						"activetextcolor" : 						{
							"expression" : ""
						}
,
						"activetextoncolor" : 						{
							"expression" : ""
						}
,
						"bordercolor" : 						{
							"expression" : ""
						}
,
						"focusbordercolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_enum" : [ "val1", "val2" ],
							"parameter_longname" : "pushtext3[1]",
							"parameter_mmax" : 1,
							"parameter_modmode" : 0,
							"parameter_shortname" : "pushtext3",
							"parameter_type" : 2
						}

					}
,
					"text" : "track2below",
					"texton" : "bam",
					"varname" : "pushtext3[1]"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-216",
					"linecount" : 8,
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 4,
					"outlettype" : [ "int", "", "", "" ],
					"patching_rect" : [ 1513.749855637550354, 927.499911546707153, 103.200001537799835, 116.0 ],
					"text" : "jk.push @button track3below @oncolor c @offcolor bl @textoff track3below @texton bam @mode t",
					"varname" : "jkpush4[1]"
				}

			}
, 			{
				"box" : 				{
					"activebgcolor" : [ 1.0, 0.5, 0.035, 1.0 ],
					"activebgoncolor" : [ 0.0, 1.0, 1.0, 1.0 ],
					"activetextcolor" : [ 0.15, 0.15, 0.15, 1.0 ],
					"bordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
					"focusbordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
					"fontsize" : 9.0,
					"id" : "obj-4",
					"maxclass" : "live.text",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1509.999855995178223, 911.249913096427917, 44.0, 15.0 ],
					"saved_attribute_attributes" : 					{
						"activebgcolor" : 						{
							"expression" : ""
						}
,
						"activebgoncolor" : 						{
							"expression" : ""
						}
,
						"activetextcolor" : 						{
							"expression" : ""
						}
,
						"activetextoncolor" : 						{
							"expression" : ""
						}
,
						"bordercolor" : 						{
							"expression" : ""
						}
,
						"focusbordercolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_enum" : [ "val1", "val2" ],
							"parameter_longname" : "pushtext4[1]",
							"parameter_mmax" : 1,
							"parameter_modmode" : 0,
							"parameter_shortname" : "pushtext4",
							"parameter_type" : 2
						}

					}
,
					"text" : "track3below",
					"texton" : "bam",
					"varname" : "pushtext4[1]"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-219",
					"linecount" : 8,
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 4,
					"outlettype" : [ "int", "", "", "" ],
					"patching_rect" : [ 1628.749844670295715, 927.499911546707153, 103.200001537799835, 116.0 ],
					"text" : "jk.push @button track4below @oncolor c @offcolor bl @textoff track4below @texton bam @mode t",
					"varname" : "jkpush5[1]"
				}

			}
, 			{
				"box" : 				{
					"activebgcolor" : [ 1.0, 0.5, 0.035, 1.0 ],
					"activebgoncolor" : [ 0.0, 1.0, 1.0, 1.0 ],
					"activetextcolor" : [ 0.15, 0.15, 0.15, 1.0 ],
					"bordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
					"focusbordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
					"fontsize" : 9.0,
					"id" : "obj-134",
					"maxclass" : "live.text",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1628.749844670295715, 911.249913096427917, 44.0, 15.0 ],
					"saved_attribute_attributes" : 					{
						"activebgcolor" : 						{
							"expression" : ""
						}
,
						"activebgoncolor" : 						{
							"expression" : ""
						}
,
						"activetextcolor" : 						{
							"expression" : ""
						}
,
						"activetextoncolor" : 						{
							"expression" : ""
						}
,
						"bordercolor" : 						{
							"expression" : ""
						}
,
						"focusbordercolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_enum" : [ "val1", "val2" ],
							"parameter_longname" : "pushtext5[1]",
							"parameter_mmax" : 1,
							"parameter_modmode" : 0,
							"parameter_shortname" : "pushtext5",
							"parameter_type" : 2
						}

					}
,
					"text" : "track4below",
					"texton" : "bam",
					"varname" : "pushtext5[1]"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-128",
					"maxclass" : "newobj",
					"numinlets" : 5,
					"numoutlets" : 5,
					"outlettype" : [ "bang", "bang", "bang", "bang", "" ],
					"patching_rect" : [ 2030.619999999999891, 713.149999999999977, 64.0, 22.0 ],
					"text" : "sel 0 1 2 3"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-119",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 2327.419371485710144, 1882.25807797908783, 97.0, 22.0 ],
					"text" : "s soprano_micro"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-118",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 2122.580660343170166, 1909.677433013916016, 73.0, 22.0 ],
					"text" : "s alto_micro"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-117",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 1954.838723659515381, 1901.612916827201843, 81.0, 22.0 ],
					"text" : "s tenor_micro"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-116",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 1773.1400000000001, 1905.3900000000001, 79.0, 22.0 ],
					"text" : "s bass_micro"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-113",
					"maxclass" : "newobj",
					"numinlets" : 6,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 2335.483887672424316, 1753.225818991661072, 134.0, 22.0 ],
					"text" : "scale 0 1000 -0.02 0.02"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-114",
					"linecount" : 15,
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 2335.483887672424316, 1785.483883738517761, 50.0, 210.0 ],
					"text" : "0.00876 0.00056 0.00196 0.00292 0.00192 0.00412 0.00556 0.0118"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-115",
					"maxclass" : "newobj",
					"numinlets" : 8,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 2335.483887672424316, 1727.41936719417572, 92.5, 22.0 ],
					"text" : "pak i i i i i i i i"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-110",
					"maxclass" : "newobj",
					"numinlets" : 6,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 2141.93549919128418, 1761.290335178375244, 134.0, 22.0 ],
					"text" : "scale 0 1000 -0.02 0.02"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-111",
					"linecount" : 13,
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 2141.93549919128418, 1793.548399925231934, 53.0, 183.0 ],
					"text" : "-0.00748 -0.00584 -0.00968 -0.00112 -0.00536 0.00368 0.00208 0.00536"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-112",
					"maxclass" : "newobj",
					"numinlets" : 8,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 2141.93549919128418, 1735.483883380889893, 92.5, 22.0 ],
					"text" : "pak i i i i i i i i"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-107",
					"maxclass" : "newobj",
					"numinlets" : 6,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1963.513382434844971, 1769.354851365089417, 134.0, 22.0 ],
					"text" : "scale 0 1000 -0.02 0.02"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-108",
					"linecount" : 15,
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 2005.513382434844971, 1801.428614377975464, 50.0, 210.0 ],
					"text" : "0.00432 0.00444 0.00116 0.0044 -0.00064 0.01064 0.01132 -0.00412"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-109",
					"maxclass" : "newobj",
					"numinlets" : 8,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1963.513382434844971, 1743.548399567604065, 92.5, 22.0 ],
					"text" : "pak i i i i i i i i"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-106",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1225.806460380554199, 24.193548560142517, 29.5, 22.0 ],
					"text" : "6"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-104",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 1227.059999999999945, 174.47999999999999, 65.0, 22.0 ],
					"text" : "s foo_var1"
				}

			}
, 			{
				"box" : 				{
					"activedialcolor" : [ 1.0, 0.0, 0.0, 1.0 ],
					"activefgdialcolor" : [ 0.0, 0.0, 1.0, 1.0 ],
					"activeneedlecolor" : [ 1.0, 1.0, 0.0, 1.0 ],
					"fgdialcolor" : [ 0.243137254901961, 0.698039215686274, 0.415686274509804, 1.0 ],
					"focusbordercolor" : [ 0.65098, 0.666667, 0.662745, 1.0 ],
					"fontname" : "Arial",
					"fontsize" : 10.0,
					"id" : "obj-103",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1760.000041961669922, 1531.428607940673828, 27.0, 48.0 ],
					"saved_attribute_attributes" : 					{
						"activedialcolor" : 						{
							"expression" : ""
						}
,
						"activefgdialcolor" : 						{
							"expression" : ""
						}
,
						"activeneedlecolor" : 						{
							"expression" : ""
						}
,
						"fgdialcolor" : 						{
							"expression" : ""
						}
,
						"focusbordercolor" : 						{
							"expression" : ""
						}
,
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "2[31]",
							"parameter_mmax" : 1000.0,
							"parameter_modmode" : 0,
							"parameter_shortname" : "2",
							"parameter_type" : 0,
							"parameter_unitstyle" : 0
						}

					}

				}

			}
, 			{
				"box" : 				{
					"activedialcolor" : [ 1.0, 0.0, 0.0, 1.0 ],
					"activefgdialcolor" : [ 0.0, 0.0, 1.0, 1.0 ],
					"activeneedlecolor" : [ 1.0, 1.0, 0.0, 1.0 ],
					"fgdialcolor" : [ 0.243137254901961, 0.698039215686274, 0.415686274509804, 1.0 ],
					"focusbordercolor" : [ 0.65098, 0.666667, 0.662745, 1.0 ],
					"fontname" : "Arial",
					"fontsize" : 10.0,
					"id" : "obj-102",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 2980.000071048736572, 1531.428607940673828, 27.0, 48.0 ],
					"saved_attribute_attributes" : 					{
						"activedialcolor" : 						{
							"expression" : ""
						}
,
						"activefgdialcolor" : 						{
							"expression" : ""
						}
,
						"activeneedlecolor" : 						{
							"expression" : ""
						}
,
						"fgdialcolor" : 						{
							"expression" : ""
						}
,
						"focusbordercolor" : 						{
							"expression" : ""
						}
,
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "2[30]",
							"parameter_mmax" : 1000.0,
							"parameter_modmode" : 0,
							"parameter_shortname" : "2",
							"parameter_type" : 0,
							"parameter_unitstyle" : 0
						}

					}
,
					"varname" : "live.dial[25]"
				}

			}
, 			{
				"box" : 				{
					"activedialcolor" : [ 1.0, 0.0, 0.0, 1.0 ],
					"activefgdialcolor" : [ 0.0, 0.0, 1.0, 1.0 ],
					"activeneedlecolor" : [ 1.0, 1.0, 0.0, 1.0 ],
					"fgdialcolor" : [ 0.243137254901961, 0.698039215686274, 0.415686274509804, 1.0 ],
					"focusbordercolor" : [ 0.65098, 0.666667, 0.662745, 1.0 ],
					"fontname" : "Arial",
					"fontsize" : 10.0,
					"id" : "obj-84",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 2901.428640604019165, 1531.428607940673828, 27.0, 48.0 ],
					"saved_attribute_attributes" : 					{
						"activedialcolor" : 						{
							"expression" : ""
						}
,
						"activefgdialcolor" : 						{
							"expression" : ""
						}
,
						"activeneedlecolor" : 						{
							"expression" : ""
						}
,
						"fgdialcolor" : 						{
							"expression" : ""
						}
,
						"focusbordercolor" : 						{
							"expression" : ""
						}
,
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "2[16]",
							"parameter_mmax" : 1000.0,
							"parameter_modmode" : 0,
							"parameter_shortname" : "2",
							"parameter_type" : 0,
							"parameter_unitstyle" : 0
						}

					}
,
					"varname" : "live.dial[12]"
				}

			}
, 			{
				"box" : 				{
					"activedialcolor" : [ 1.0, 0.0, 0.0, 1.0 ],
					"activefgdialcolor" : [ 0.0, 0.0, 1.0, 1.0 ],
					"activeneedlecolor" : [ 1.0, 1.0, 0.0, 1.0 ],
					"fgdialcolor" : [ 0.243137254901961, 0.698039215686274, 0.415686274509804, 1.0 ],
					"focusbordercolor" : [ 0.65098, 0.666667, 0.662745, 1.0 ],
					"fontname" : "Arial",
					"fontsize" : 10.0,
					"id" : "obj-85",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 2822.857210159301758, 1531.428607940673828, 27.0, 48.0 ],
					"saved_attribute_attributes" : 					{
						"activedialcolor" : 						{
							"expression" : ""
						}
,
						"activefgdialcolor" : 						{
							"expression" : ""
						}
,
						"activeneedlecolor" : 						{
							"expression" : ""
						}
,
						"fgdialcolor" : 						{
							"expression" : ""
						}
,
						"focusbordercolor" : 						{
							"expression" : ""
						}
,
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "2[17]",
							"parameter_mmax" : 1000.0,
							"parameter_modmode" : 0,
							"parameter_shortname" : "2",
							"parameter_type" : 0,
							"parameter_unitstyle" : 0
						}

					}
,
					"varname" : "live.dial[13]"
				}

			}
, 			{
				"box" : 				{
					"activedialcolor" : [ 1.0, 0.0, 0.0, 1.0 ],
					"activefgdialcolor" : [ 0.0, 0.0, 1.0, 1.0 ],
					"activeneedlecolor" : [ 1.0, 1.0, 0.0, 1.0 ],
					"fgdialcolor" : [ 0.243137254901961, 0.698039215686274, 0.415686274509804, 1.0 ],
					"focusbordercolor" : [ 0.65098, 0.666667, 0.662745, 1.0 ],
					"fontname" : "Arial",
					"fontsize" : 10.0,
					"id" : "obj-86",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 2744.285779714584351, 1531.428607940673828, 27.0, 48.0 ],
					"saved_attribute_attributes" : 					{
						"activedialcolor" : 						{
							"expression" : ""
						}
,
						"activefgdialcolor" : 						{
							"expression" : ""
						}
,
						"activeneedlecolor" : 						{
							"expression" : ""
						}
,
						"fgdialcolor" : 						{
							"expression" : ""
						}
,
						"focusbordercolor" : 						{
							"expression" : ""
						}
,
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "2[18]",
							"parameter_mmax" : 1000.0,
							"parameter_modmode" : 0,
							"parameter_shortname" : "2",
							"parameter_type" : 0,
							"parameter_unitstyle" : 0
						}

					}

				}

			}
, 			{
				"box" : 				{
					"fontsize" : 10.0,
					"id" : "obj-87",
					"linecount" : 4,
					"maxclass" : "newobj",
					"numinlets" : 6,
					"numoutlets" : 4,
					"outlettype" : [ "float", "float", "float", "float" ],
					"patching_rect" : [ 2744.067862033843539, 1472.580655694007874, 265.0, 53.0 ],
					"text" : "jk.pushrotaryAlt 10 @dial1 r @dial2 b @init 500 @needle y @dialtextcolor k @bton y @btoff b @oncolor y @offcolor b @texton \"3\" @textoff \"Rotary 3\" @lo 0 @hi 1000 @inc 1"
				}

			}
, 			{
				"box" : 				{
					"activedialcolor" : [ 1.0, 0.0, 0.0, 1.0 ],
					"activefgdialcolor" : [ 0.0, 0.0, 1.0, 1.0 ],
					"activeneedlecolor" : [ 1.0, 1.0, 0.0, 1.0 ],
					"fgdialcolor" : [ 0.243137254901961, 0.698039215686274, 0.415686274509804, 1.0 ],
					"focusbordercolor" : [ 0.65098, 0.666667, 0.662745, 1.0 ],
					"fontname" : "Arial",
					"fontsize" : 10.0,
					"id" : "obj-88",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 2651.428634643554688, 1531.428607940673828, 27.0, 48.0 ],
					"saved_attribute_attributes" : 					{
						"activedialcolor" : 						{
							"expression" : ""
						}
,
						"activefgdialcolor" : 						{
							"expression" : ""
						}
,
						"activeneedlecolor" : 						{
							"expression" : ""
						}
,
						"fgdialcolor" : 						{
							"expression" : ""
						}
,
						"focusbordercolor" : 						{
							"expression" : ""
						}
,
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "2[19]",
							"parameter_mmax" : 1000.0,
							"parameter_modmode" : 0,
							"parameter_shortname" : "2",
							"parameter_type" : 0,
							"parameter_unitstyle" : 0
						}

					}
,
					"varname" : "live.dial[14]"
				}

			}
, 			{
				"box" : 				{
					"activedialcolor" : [ 1.0, 0.0, 0.0, 1.0 ],
					"activefgdialcolor" : [ 0.0, 0.0, 1.0, 1.0 ],
					"activeneedlecolor" : [ 1.0, 1.0, 0.0, 1.0 ],
					"fgdialcolor" : [ 0.243137254901961, 0.698039215686274, 0.415686274509804, 1.0 ],
					"focusbordercolor" : [ 0.65098, 0.666667, 0.662745, 1.0 ],
					"fontname" : "Arial",
					"fontsize" : 10.0,
					"id" : "obj-89",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 2571.428632736206055, 1531.428607940673828, 27.0, 48.0 ],
					"saved_attribute_attributes" : 					{
						"activedialcolor" : 						{
							"expression" : ""
						}
,
						"activefgdialcolor" : 						{
							"expression" : ""
						}
,
						"activeneedlecolor" : 						{
							"expression" : ""
						}
,
						"fgdialcolor" : 						{
							"expression" : ""
						}
,
						"focusbordercolor" : 						{
							"expression" : ""
						}
,
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "2[20]",
							"parameter_mmax" : 1000.0,
							"parameter_modmode" : 0,
							"parameter_shortname" : "2",
							"parameter_type" : 0,
							"parameter_unitstyle" : 0
						}

					}
,
					"varname" : "live.dial[15]"
				}

			}
, 			{
				"box" : 				{
					"activedialcolor" : [ 1.0, 0.0, 0.0, 1.0 ],
					"activefgdialcolor" : [ 0.0, 0.0, 1.0, 1.0 ],
					"activeneedlecolor" : [ 1.0, 1.0, 0.0, 1.0 ],
					"fgdialcolor" : [ 0.243137254901961, 0.698039215686274, 0.415686274509804, 1.0 ],
					"focusbordercolor" : [ 0.65098, 0.666667, 0.662745, 1.0 ],
					"fontname" : "Arial",
					"fontsize" : 10.0,
					"id" : "obj-90",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 2491.428630828857422, 1531.428607940673828, 27.0, 48.0 ],
					"saved_attribute_attributes" : 					{
						"activedialcolor" : 						{
							"expression" : ""
						}
,
						"activefgdialcolor" : 						{
							"expression" : ""
						}
,
						"activeneedlecolor" : 						{
							"expression" : ""
						}
,
						"fgdialcolor" : 						{
							"expression" : ""
						}
,
						"focusbordercolor" : 						{
							"expression" : ""
						}
,
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "2[21]",
							"parameter_mmax" : 1000.0,
							"parameter_modmode" : 0,
							"parameter_shortname" : "2",
							"parameter_type" : 0,
							"parameter_unitstyle" : 0
						}

					}
,
					"varname" : "live.dial[16]"
				}

			}
, 			{
				"box" : 				{
					"activedialcolor" : [ 1.0, 0.0, 0.0, 1.0 ],
					"activefgdialcolor" : [ 0.0, 0.0, 1.0, 1.0 ],
					"activeneedlecolor" : [ 1.0, 1.0, 0.0, 1.0 ],
					"fgdialcolor" : [ 0.243137254901961, 0.698039215686274, 0.415686274509804, 1.0 ],
					"focusbordercolor" : [ 0.65098, 0.666667, 0.662745, 1.0 ],
					"fontname" : "Arial",
					"fontsize" : 10.0,
					"id" : "obj-91",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 2411.428628921508789, 1531.428607940673828, 27.0, 48.0 ],
					"saved_attribute_attributes" : 					{
						"activedialcolor" : 						{
							"expression" : ""
						}
,
						"activefgdialcolor" : 						{
							"expression" : ""
						}
,
						"activeneedlecolor" : 						{
							"expression" : ""
						}
,
						"fgdialcolor" : 						{
							"expression" : ""
						}
,
						"focusbordercolor" : 						{
							"expression" : ""
						}
,
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "2[22]",
							"parameter_mmax" : 1000.0,
							"parameter_modmode" : 0,
							"parameter_shortname" : "2",
							"parameter_type" : 0,
							"parameter_unitstyle" : 0
						}

					}

				}

			}
, 			{
				"box" : 				{
					"fontsize" : 10.0,
					"id" : "obj-92",
					"linecount" : 4,
					"maxclass" : "newobj",
					"numinlets" : 6,
					"numoutlets" : 4,
					"outlettype" : [ "float", "float", "float", "float" ],
					"patching_rect" : [ 2415.819266637166038, 1472.580655694007874, 262.711870670318604, 53.0 ],
					"text" : "jk.pushrotaryAlt 9 @dial1 r @dial2 b @init 500 @needle y @dialtextcolor k @bton y @btoff b @oncolor y @offcolor b @texton \"3\" @textoff \"Rotary 3\" @lo 0 @hi 1000 @inc 1"
				}

			}
, 			{
				"box" : 				{
					"activedialcolor" : [ 1.0, 0.0, 0.0, 1.0 ],
					"activefgdialcolor" : [ 0.0, 0.0, 1.0, 1.0 ],
					"activeneedlecolor" : [ 1.0, 1.0, 0.0, 1.0 ],
					"fgdialcolor" : [ 0.243137254901961, 0.698039215686274, 0.415686274509804, 1.0 ],
					"focusbordercolor" : [ 0.65098, 0.666667, 0.662745, 1.0 ],
					"fontname" : "Arial",
					"fontsize" : 10.0,
					"id" : "obj-93",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 2322.857198238372803, 1531.428607940673828, 27.0, 48.0 ],
					"saved_attribute_attributes" : 					{
						"activedialcolor" : 						{
							"expression" : ""
						}
,
						"activefgdialcolor" : 						{
							"expression" : ""
						}
,
						"activeneedlecolor" : 						{
							"expression" : ""
						}
,
						"fgdialcolor" : 						{
							"expression" : ""
						}
,
						"focusbordercolor" : 						{
							"expression" : ""
						}
,
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "2[23]",
							"parameter_mmax" : 1000.0,
							"parameter_modmode" : 0,
							"parameter_shortname" : "2",
							"parameter_type" : 0,
							"parameter_unitstyle" : 0
						}

					}
,
					"varname" : "live.dial[18]"
				}

			}
, 			{
				"box" : 				{
					"activedialcolor" : [ 1.0, 0.0, 0.0, 1.0 ],
					"activefgdialcolor" : [ 0.0, 0.0, 1.0, 1.0 ],
					"activeneedlecolor" : [ 1.0, 1.0, 0.0, 1.0 ],
					"fgdialcolor" : [ 0.243137254901961, 0.698039215686274, 0.415686274509804, 1.0 ],
					"focusbordercolor" : [ 0.65098, 0.666667, 0.662745, 1.0 ],
					"fontname" : "Arial",
					"fontsize" : 10.0,
					"id" : "obj-94",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 2244.285767793655396, 1531.428607940673828, 27.0, 48.0 ],
					"saved_attribute_attributes" : 					{
						"activedialcolor" : 						{
							"expression" : ""
						}
,
						"activefgdialcolor" : 						{
							"expression" : ""
						}
,
						"activeneedlecolor" : 						{
							"expression" : ""
						}
,
						"fgdialcolor" : 						{
							"expression" : ""
						}
,
						"focusbordercolor" : 						{
							"expression" : ""
						}
,
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "2[24]",
							"parameter_mmax" : 1000.0,
							"parameter_modmode" : 0,
							"parameter_shortname" : "2",
							"parameter_type" : 0,
							"parameter_unitstyle" : 0
						}

					}
,
					"varname" : "live.dial[19]"
				}

			}
, 			{
				"box" : 				{
					"activedialcolor" : [ 1.0, 0.0, 0.0, 1.0 ],
					"activefgdialcolor" : [ 0.0, 0.0, 1.0, 1.0 ],
					"activeneedlecolor" : [ 1.0, 1.0, 0.0, 1.0 ],
					"fgdialcolor" : [ 0.243137254901961, 0.698039215686274, 0.415686274509804, 1.0 ],
					"focusbordercolor" : [ 0.65098, 0.666667, 0.662745, 1.0 ],
					"fontname" : "Arial",
					"fontsize" : 10.0,
					"id" : "obj-95",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 2165.714337348937988, 1531.428607940673828, 27.0, 48.0 ],
					"saved_attribute_attributes" : 					{
						"activedialcolor" : 						{
							"expression" : ""
						}
,
						"activefgdialcolor" : 						{
							"expression" : ""
						}
,
						"activeneedlecolor" : 						{
							"expression" : ""
						}
,
						"fgdialcolor" : 						{
							"expression" : ""
						}
,
						"focusbordercolor" : 						{
							"expression" : ""
						}
,
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "2[25]",
							"parameter_mmax" : 1000.0,
							"parameter_modmode" : 0,
							"parameter_shortname" : "2",
							"parameter_type" : 0,
							"parameter_unitstyle" : 0
						}

					}
,
					"varname" : "live.dial[20]"
				}

			}
, 			{
				"box" : 				{
					"activedialcolor" : [ 1.0, 0.0, 0.0, 1.0 ],
					"activefgdialcolor" : [ 0.0, 0.0, 1.0, 1.0 ],
					"activeneedlecolor" : [ 1.0, 1.0, 0.0, 1.0 ],
					"fgdialcolor" : [ 0.243137254901961, 0.698039215686274, 0.415686274509804, 1.0 ],
					"focusbordercolor" : [ 0.65098, 0.666667, 0.662745, 1.0 ],
					"fontname" : "Arial",
					"fontsize" : 10.0,
					"id" : "obj-96",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 2087.142906904220581, 1531.428607940673828, 27.0, 48.0 ],
					"saved_attribute_attributes" : 					{
						"activedialcolor" : 						{
							"expression" : ""
						}
,
						"activefgdialcolor" : 						{
							"expression" : ""
						}
,
						"activeneedlecolor" : 						{
							"expression" : ""
						}
,
						"fgdialcolor" : 						{
							"expression" : ""
						}
,
						"focusbordercolor" : 						{
							"expression" : ""
						}
,
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "2[26]",
							"parameter_mmax" : 1000.0,
							"parameter_modmode" : 0,
							"parameter_shortname" : "2",
							"parameter_type" : 0,
							"parameter_unitstyle" : 0
						}

					}

				}

			}
, 			{
				"box" : 				{
					"fontsize" : 10.0,
					"id" : "obj-97",
					"linecount" : 4,
					"maxclass" : "newobj",
					"numinlets" : 6,
					"numoutlets" : 4,
					"outlettype" : [ "float", "float", "float", "float" ],
					"patching_rect" : [ 2087.570671240488537, 1472.580655694007874, 262.711870670318604, 53.0 ],
					"text" : "jk.pushrotaryAlt 8 @dial1 r @dial2 b @init 500 @needle y @dialtextcolor k @bton y @btoff b @oncolor y @offcolor b @texton \"3\" @textoff \"Rotary 3\" @lo 0 @hi 1000 @inc 1"
				}

			}
, 			{
				"box" : 				{
					"activedialcolor" : [ 1.0, 0.0, 0.0, 1.0 ],
					"activefgdialcolor" : [ 0.0, 0.0, 1.0, 1.0 ],
					"activeneedlecolor" : [ 1.0, 1.0, 0.0, 1.0 ],
					"fgdialcolor" : [ 0.243137254901961, 0.698039215686274, 0.415686274509804, 1.0 ],
					"focusbordercolor" : [ 0.65098, 0.666667, 0.662745, 1.0 ],
					"fontname" : "Arial",
					"fontsize" : 10.0,
					"id" : "obj-98",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1995.714333295822144, 1531.428607940673828, 27.0, 48.0 ],
					"saved_attribute_attributes" : 					{
						"activedialcolor" : 						{
							"expression" : ""
						}
,
						"activefgdialcolor" : 						{
							"expression" : ""
						}
,
						"activeneedlecolor" : 						{
							"expression" : ""
						}
,
						"fgdialcolor" : 						{
							"expression" : ""
						}
,
						"focusbordercolor" : 						{
							"expression" : ""
						}
,
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "2[27]",
							"parameter_mmax" : 1000.0,
							"parameter_modmode" : 0,
							"parameter_shortname" : "2",
							"parameter_type" : 0,
							"parameter_unitstyle" : 0
						}

					}
,
					"varname" : "live.dial[22]"
				}

			}
, 			{
				"box" : 				{
					"activedialcolor" : [ 1.0, 0.0, 0.0, 1.0 ],
					"activefgdialcolor" : [ 0.0, 0.0, 1.0, 1.0 ],
					"activeneedlecolor" : [ 1.0, 1.0, 0.0, 1.0 ],
					"fgdialcolor" : [ 0.243137254901961, 0.698039215686274, 0.415686274509804, 1.0 ],
					"focusbordercolor" : [ 0.65098, 0.666667, 0.662745, 1.0 ],
					"fontname" : "Arial",
					"fontsize" : 10.0,
					"id" : "obj-99",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1917.142902851104736, 1531.428607940673828, 27.0, 48.0 ],
					"saved_attribute_attributes" : 					{
						"activedialcolor" : 						{
							"expression" : ""
						}
,
						"activefgdialcolor" : 						{
							"expression" : ""
						}
,
						"activeneedlecolor" : 						{
							"expression" : ""
						}
,
						"fgdialcolor" : 						{
							"expression" : ""
						}
,
						"focusbordercolor" : 						{
							"expression" : ""
						}
,
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "2[28]",
							"parameter_mmax" : 1000.0,
							"parameter_modmode" : 0,
							"parameter_shortname" : "2",
							"parameter_type" : 0,
							"parameter_unitstyle" : 0
						}

					}
,
					"varname" : "live.dial[23]"
				}

			}
, 			{
				"box" : 				{
					"activedialcolor" : [ 1.0, 0.0, 0.0, 1.0 ],
					"activefgdialcolor" : [ 0.0, 0.0, 1.0, 1.0 ],
					"activeneedlecolor" : [ 1.0, 1.0, 0.0, 1.0 ],
					"fgdialcolor" : [ 0.243137254901961, 0.698039215686274, 0.415686274509804, 1.0 ],
					"focusbordercolor" : [ 0.65098, 0.666667, 0.662745, 1.0 ],
					"fontname" : "Arial",
					"fontsize" : 10.0,
					"id" : "obj-100",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1838.571472406387329, 1531.428607940673828, 27.0, 48.0 ],
					"saved_attribute_attributes" : 					{
						"activedialcolor" : 						{
							"expression" : ""
						}
,
						"activefgdialcolor" : 						{
							"expression" : ""
						}
,
						"activeneedlecolor" : 						{
							"expression" : ""
						}
,
						"fgdialcolor" : 						{
							"expression" : ""
						}
,
						"focusbordercolor" : 						{
							"expression" : ""
						}
,
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "2[29]",
							"parameter_mmax" : 1000.0,
							"parameter_modmode" : 0,
							"parameter_shortname" : "2",
							"parameter_type" : 0,
							"parameter_unitstyle" : 0
						}

					}
,
					"varname" : "live.dial[24]"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 10.0,
					"id" : "obj-101",
					"linecount" : 4,
					"maxclass" : "newobj",
					"numinlets" : 6,
					"numoutlets" : 4,
					"outlettype" : [ "float", "float", "float", "float" ],
					"patching_rect" : [ 1759.322075843811035, 1472.580655694007874, 262.711870670318604, 53.0 ],
					"text" : "jk.pushrotaryAlt 7 @dial1 r @dial2 b @init 500 @needle y @dialtextcolor k @bton y @btoff b @oncolor y @offcolor b @texton \"3\" @textoff \"Rotary 3\" @lo 0 @hi 1000 @inc 1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-83",
					"maxclass" : "newobj",
					"numinlets" : 6,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1775.806464314460754, 1775.806464314460754, 134.0, 22.0 ],
					"text" : "scale 0 1000 -0.02 0.02"
				}

			}
, 			{
				"box" : 				{
					"activedialcolor" : [ 1.0, 0.0, 0.0, 1.0 ],
					"activefgdialcolor" : [ 0.0, 0.0, 1.0, 1.0 ],
					"activeneedlecolor" : [ 1.0, 1.0, 0.0, 1.0 ],
					"fgdialcolor" : [ 0.243137254901961, 0.698039215686274, 0.415686274509804, 1.0 ],
					"focusbordercolor" : [ 0.65098, 0.666667, 0.662745, 1.0 ],
					"fontname" : "Arial",
					"fontsize" : 10.0,
					"id" : "obj-78",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 2991.428642749786377, 1314.285745620727539, 27.0, 48.0 ],
					"saved_attribute_attributes" : 					{
						"activedialcolor" : 						{
							"expression" : ""
						}
,
						"activefgdialcolor" : 						{
							"expression" : ""
						}
,
						"activeneedlecolor" : 						{
							"expression" : ""
						}
,
						"fgdialcolor" : 						{
							"expression" : ""
						}
,
						"focusbordercolor" : 						{
							"expression" : ""
						}
,
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "2[12]",
							"parameter_mmax" : 1000.0,
							"parameter_modmode" : 0,
							"parameter_shortname" : "2",
							"parameter_type" : 0,
							"parameter_unitstyle" : 0
						}

					}
,
					"varname" : "live.dial[4]"
				}

			}
, 			{
				"box" : 				{
					"activedialcolor" : [ 1.0, 0.0, 0.0, 1.0 ],
					"activefgdialcolor" : [ 0.0, 0.0, 1.0, 1.0 ],
					"activeneedlecolor" : [ 1.0, 1.0, 0.0, 1.0 ],
					"fgdialcolor" : [ 0.243137254901961, 0.698039215686274, 0.415686274509804, 1.0 ],
					"focusbordercolor" : [ 0.65098, 0.666667, 0.662745, 1.0 ],
					"fontname" : "Arial",
					"fontsize" : 10.0,
					"id" : "obj-79",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 2911.428640842437744, 1314.285745620727539, 27.0, 48.0 ],
					"saved_attribute_attributes" : 					{
						"activedialcolor" : 						{
							"expression" : ""
						}
,
						"activefgdialcolor" : 						{
							"expression" : ""
						}
,
						"activeneedlecolor" : 						{
							"expression" : ""
						}
,
						"fgdialcolor" : 						{
							"expression" : ""
						}
,
						"focusbordercolor" : 						{
							"expression" : ""
						}
,
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "2[13]",
							"parameter_mmax" : 1000.0,
							"parameter_modmode" : 0,
							"parameter_shortname" : "2",
							"parameter_type" : 0,
							"parameter_unitstyle" : 0
						}

					}
,
					"varname" : "live.dial[5]"
				}

			}
, 			{
				"box" : 				{
					"activedialcolor" : [ 1.0, 0.0, 0.0, 1.0 ],
					"activefgdialcolor" : [ 0.0, 0.0, 1.0, 1.0 ],
					"activeneedlecolor" : [ 1.0, 1.0, 0.0, 1.0 ],
					"fgdialcolor" : [ 0.243137254901961, 0.698039215686274, 0.415686274509804, 1.0 ],
					"focusbordercolor" : [ 0.65098, 0.666667, 0.662745, 1.0 ],
					"fontname" : "Arial",
					"fontsize" : 10.0,
					"id" : "obj-80",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 2832.857210397720337, 1314.285745620727539, 27.0, 48.0 ],
					"saved_attribute_attributes" : 					{
						"activedialcolor" : 						{
							"expression" : ""
						}
,
						"activefgdialcolor" : 						{
							"expression" : ""
						}
,
						"activeneedlecolor" : 						{
							"expression" : ""
						}
,
						"fgdialcolor" : 						{
							"expression" : ""
						}
,
						"focusbordercolor" : 						{
							"expression" : ""
						}
,
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "2[14]",
							"parameter_mmax" : 1000.0,
							"parameter_modmode" : 0,
							"parameter_shortname" : "2",
							"parameter_type" : 0,
							"parameter_unitstyle" : 0
						}

					}
,
					"varname" : "live.dial[8]"
				}

			}
, 			{
				"box" : 				{
					"activedialcolor" : [ 1.0, 0.0, 0.0, 1.0 ],
					"activefgdialcolor" : [ 0.0, 0.0, 1.0, 1.0 ],
					"activeneedlecolor" : [ 1.0, 1.0, 0.0, 1.0 ],
					"fgdialcolor" : [ 0.243137254901961, 0.698039215686274, 0.415686274509804, 1.0 ],
					"focusbordercolor" : [ 0.65098, 0.666667, 0.662745, 1.0 ],
					"fontname" : "Arial",
					"fontsize" : 10.0,
					"id" : "obj-81",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 2752.857208490371704, 1314.285745620727539, 27.0, 48.0 ],
					"saved_attribute_attributes" : 					{
						"activedialcolor" : 						{
							"expression" : ""
						}
,
						"activefgdialcolor" : 						{
							"expression" : ""
						}
,
						"activeneedlecolor" : 						{
							"expression" : ""
						}
,
						"fgdialcolor" : 						{
							"expression" : ""
						}
,
						"focusbordercolor" : 						{
							"expression" : ""
						}
,
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "2[15]",
							"parameter_mmax" : 1000.0,
							"parameter_modmode" : 0,
							"parameter_shortname" : "2",
							"parameter_type" : 0,
							"parameter_unitstyle" : 0
						}

					}

				}

			}
, 			{
				"box" : 				{
					"fontsize" : 10.0,
					"id" : "obj-82",
					"linecount" : 4,
					"maxclass" : "newobj",
					"numinlets" : 6,
					"numoutlets" : 4,
					"outlettype" : [ "float", "float", "float", "float" ],
					"patching_rect" : [ 2752.542438507080078, 1251.928559184074402, 262.711870670318604, 53.0 ],
					"text" : "jk.pushrotaryAlt 6 @dial1 r @dial2 b @init 500 @needle y @dialtextcolor k @bton y @btoff b @oncolor y @offcolor b @texton \"3\" @textoff \"Rotary 3\" @lo 0 @hi 1000 @inc 1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-77",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 1963.513382434844971, 1217.56748628616333, 24.0, 24.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-74",
					"linecount" : 9,
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1630.645172953605652, 1793.548399925231934, 55.0, 129.0 ],
					"text" : "0.00152 -0.01324 0.00008 0.00184 0.00032 0.00512 0.00452 0.00484"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-72",
					"maxclass" : "newobj",
					"numinlets" : 8,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 1775.806464314460754, 1750.000012516975403, 92.5, 22.0 ],
					"text" : "pak i i i i i i i i"
				}

			}
, 			{
				"box" : 				{
					"activedialcolor" : [ 1.0, 0.0, 0.0, 1.0 ],
					"activefgdialcolor" : [ 0.0, 0.0, 1.0, 1.0 ],
					"activeneedlecolor" : [ 1.0, 1.0, 0.0, 1.0 ],
					"fgdialcolor" : [ 0.243137254901961, 0.698039215686274, 0.415686274509804, 1.0 ],
					"focusbordercolor" : [ 0.65098, 0.666667, 0.662745, 1.0 ],
					"fontname" : "Arial",
					"fontsize" : 10.0,
					"id" : "obj-63",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 2657.14292049407959, 1314.285745620727539, 27.0, 48.0 ],
					"saved_attribute_attributes" : 					{
						"activedialcolor" : 						{
							"expression" : ""
						}
,
						"activefgdialcolor" : 						{
							"expression" : ""
						}
,
						"activeneedlecolor" : 						{
							"expression" : ""
						}
,
						"fgdialcolor" : 						{
							"expression" : ""
						}
,
						"focusbordercolor" : 						{
							"expression" : ""
						}
,
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "2[8]",
							"parameter_mmax" : 1000.0,
							"parameter_modmode" : 0,
							"parameter_shortname" : "2",
							"parameter_type" : 0,
							"parameter_unitstyle" : 0
						}

					}
,
					"varname" : "live.dial[9]"
				}

			}
, 			{
				"box" : 				{
					"activedialcolor" : [ 1.0, 0.0, 0.0, 1.0 ],
					"activefgdialcolor" : [ 0.0, 0.0, 1.0, 1.0 ],
					"activeneedlecolor" : [ 1.0, 1.0, 0.0, 1.0 ],
					"fgdialcolor" : [ 0.243137254901961, 0.698039215686274, 0.415686274509804, 1.0 ],
					"focusbordercolor" : [ 0.65098, 0.666667, 0.662745, 1.0 ],
					"fontname" : "Arial",
					"fontsize" : 10.0,
					"id" : "obj-64",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 2578.571490049362183, 1314.285745620727539, 27.0, 48.0 ],
					"saved_attribute_attributes" : 					{
						"activedialcolor" : 						{
							"expression" : ""
						}
,
						"activefgdialcolor" : 						{
							"expression" : ""
						}
,
						"activeneedlecolor" : 						{
							"expression" : ""
						}
,
						"fgdialcolor" : 						{
							"expression" : ""
						}
,
						"focusbordercolor" : 						{
							"expression" : ""
						}
,
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "2[9]",
							"parameter_mmax" : 1000.0,
							"parameter_modmode" : 0,
							"parameter_shortname" : "2",
							"parameter_type" : 0,
							"parameter_unitstyle" : 0
						}

					}
,
					"varname" : "live.dial[10]"
				}

			}
, 			{
				"box" : 				{
					"activedialcolor" : [ 1.0, 0.0, 0.0, 1.0 ],
					"activefgdialcolor" : [ 0.0, 0.0, 1.0, 1.0 ],
					"activeneedlecolor" : [ 1.0, 1.0, 0.0, 1.0 ],
					"fgdialcolor" : [ 0.243137254901961, 0.698039215686274, 0.415686274509804, 1.0 ],
					"focusbordercolor" : [ 0.65098, 0.666667, 0.662745, 1.0 ],
					"fontname" : "Arial",
					"fontsize" : 10.0,
					"id" : "obj-65",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 2498.57148814201355, 1314.285745620727539, 27.0, 48.0 ],
					"saved_attribute_attributes" : 					{
						"activedialcolor" : 						{
							"expression" : ""
						}
,
						"activefgdialcolor" : 						{
							"expression" : ""
						}
,
						"activeneedlecolor" : 						{
							"expression" : ""
						}
,
						"fgdialcolor" : 						{
							"expression" : ""
						}
,
						"focusbordercolor" : 						{
							"expression" : ""
						}
,
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "2[10]",
							"parameter_mmax" : 1000.0,
							"parameter_modmode" : 0,
							"parameter_shortname" : "2",
							"parameter_type" : 0,
							"parameter_unitstyle" : 0
						}

					}
,
					"varname" : "live.dial[11]"
				}

			}
, 			{
				"box" : 				{
					"activedialcolor" : [ 1.0, 0.0, 0.0, 1.0 ],
					"activefgdialcolor" : [ 0.0, 0.0, 1.0, 1.0 ],
					"activeneedlecolor" : [ 1.0, 1.0, 0.0, 1.0 ],
					"fgdialcolor" : [ 0.243137254901961, 0.698039215686274, 0.415686274509804, 1.0 ],
					"focusbordercolor" : [ 0.65098, 0.666667, 0.662745, 1.0 ],
					"fontname" : "Arial",
					"fontsize" : 10.0,
					"id" : "obj-66",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 2420.000057697296143, 1314.285745620727539, 27.0, 48.0 ],
					"saved_attribute_attributes" : 					{
						"activedialcolor" : 						{
							"expression" : ""
						}
,
						"activefgdialcolor" : 						{
							"expression" : ""
						}
,
						"activeneedlecolor" : 						{
							"expression" : ""
						}
,
						"fgdialcolor" : 						{
							"expression" : ""
						}
,
						"focusbordercolor" : 						{
							"expression" : ""
						}
,
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "2[11]",
							"parameter_mmax" : 1000.0,
							"parameter_modmode" : 0,
							"parameter_shortname" : "2",
							"parameter_type" : 0,
							"parameter_unitstyle" : 0
						}

					}

				}

			}
, 			{
				"box" : 				{
					"fontsize" : 10.0,
					"id" : "obj-67",
					"linecount" : 4,
					"maxclass" : "newobj",
					"numinlets" : 6,
					"numoutlets" : 4,
					"outlettype" : [ "float", "float", "float", "float" ],
					"patching_rect" : [ 2422.033956050872803, 1251.928559184074402, 262.711870670318604, 53.0 ],
					"text" : "jk.pushrotaryAlt 5 @dial1 r @dial2 b @init 500 @needle y @dialtextcolor k @bton y @btoff b @oncolor y @offcolor b @texton \"3\" @textoff \"Rotary 3\" @lo 0 @hi 1000 @inc 1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-58",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "int" ],
					"patching_rect" : [ 2161.428622961044312, 1168.571456432342529, 29.5, 22.0 ],
					"text" : "int"
				}

			}
, 			{
				"box" : 				{
					"activedialcolor" : [ 1.0, 0.0, 0.0, 1.0 ],
					"activefgdialcolor" : [ 0.0, 0.0, 1.0, 1.0 ],
					"activeneedlecolor" : [ 1.0, 1.0, 0.0, 1.0 ],
					"fgdialcolor" : [ 0.243137254901961, 0.698039215686274, 0.415686274509804, 1.0 ],
					"focusbordercolor" : [ 0.65098, 0.666667, 0.662745, 1.0 ],
					"fontname" : "Arial",
					"fontsize" : 10.0,
					"id" : "obj-48",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 2327.142912626266479, 1314.285745620727539, 27.0, 48.0 ],
					"saved_attribute_attributes" : 					{
						"activedialcolor" : 						{
							"expression" : ""
						}
,
						"activefgdialcolor" : 						{
							"expression" : ""
						}
,
						"activeneedlecolor" : 						{
							"expression" : ""
						}
,
						"fgdialcolor" : 						{
							"expression" : ""
						}
,
						"focusbordercolor" : 						{
							"expression" : ""
						}
,
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "2[4]",
							"parameter_mmax" : 1000.0,
							"parameter_modmode" : 0,
							"parameter_shortname" : "2",
							"parameter_type" : 0,
							"parameter_unitstyle" : 0
						}

					}
,
					"varname" : "live.dial[17]"
				}

			}
, 			{
				"box" : 				{
					"activedialcolor" : [ 1.0, 0.0, 0.0, 1.0 ],
					"activefgdialcolor" : [ 0.0, 0.0, 1.0, 1.0 ],
					"activeneedlecolor" : [ 1.0, 1.0, 0.0, 1.0 ],
					"fgdialcolor" : [ 0.243137254901961, 0.698039215686274, 0.415686274509804, 1.0 ],
					"focusbordercolor" : [ 0.65098, 0.666667, 0.662745, 1.0 ],
					"fontname" : "Arial",
					"fontsize" : 10.0,
					"id" : "obj-49",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 2248.571482181549072, 1314.285745620727539, 27.0, 48.0 ],
					"saved_attribute_attributes" : 					{
						"activedialcolor" : 						{
							"expression" : ""
						}
,
						"activefgdialcolor" : 						{
							"expression" : ""
						}
,
						"activeneedlecolor" : 						{
							"expression" : ""
						}
,
						"fgdialcolor" : 						{
							"expression" : ""
						}
,
						"focusbordercolor" : 						{
							"expression" : ""
						}
,
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "2[5]",
							"parameter_mmax" : 1000.0,
							"parameter_modmode" : 0,
							"parameter_shortname" : "2",
							"parameter_type" : 0,
							"parameter_unitstyle" : 0
						}

					}
,
					"varname" : "live.dial[21]"
				}

			}
, 			{
				"box" : 				{
					"activedialcolor" : [ 1.0, 0.0, 0.0, 1.0 ],
					"activefgdialcolor" : [ 0.0, 0.0, 1.0, 1.0 ],
					"activeneedlecolor" : [ 1.0, 1.0, 0.0, 1.0 ],
					"fgdialcolor" : [ 0.243137254901961, 0.698039215686274, 0.415686274509804, 1.0 ],
					"focusbordercolor" : [ 0.65098, 0.666667, 0.662745, 1.0 ],
					"fontname" : "Arial",
					"fontsize" : 10.0,
					"id" : "obj-50",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 2170.000051736831665, 1314.285745620727539, 27.0, 48.0 ],
					"saved_attribute_attributes" : 					{
						"activedialcolor" : 						{
							"expression" : ""
						}
,
						"activefgdialcolor" : 						{
							"expression" : ""
						}
,
						"activeneedlecolor" : 						{
							"expression" : ""
						}
,
						"fgdialcolor" : 						{
							"expression" : ""
						}
,
						"focusbordercolor" : 						{
							"expression" : ""
						}
,
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "2[6]",
							"parameter_mmax" : 1000.0,
							"parameter_modmode" : 0,
							"parameter_shortname" : "2",
							"parameter_type" : 0,
							"parameter_unitstyle" : 0
						}

					}
,
					"varname" : "live.dial[26]"
				}

			}
, 			{
				"box" : 				{
					"activedialcolor" : [ 1.0, 0.0, 0.0, 1.0 ],
					"activefgdialcolor" : [ 0.0, 0.0, 1.0, 1.0 ],
					"activeneedlecolor" : [ 1.0, 1.0, 0.0, 1.0 ],
					"fgdialcolor" : [ 0.243137254901961, 0.698039215686274, 0.415686274509804, 1.0 ],
					"focusbordercolor" : [ 0.65098, 0.666667, 0.662745, 1.0 ],
					"fontname" : "Arial",
					"fontsize" : 10.0,
					"id" : "obj-51",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 2091.428621292114258, 1314.285745620727539, 27.0, 48.0 ],
					"saved_attribute_attributes" : 					{
						"activedialcolor" : 						{
							"expression" : ""
						}
,
						"activefgdialcolor" : 						{
							"expression" : ""
						}
,
						"activeneedlecolor" : 						{
							"expression" : ""
						}
,
						"fgdialcolor" : 						{
							"expression" : ""
						}
,
						"focusbordercolor" : 						{
							"expression" : ""
						}
,
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "2[7]",
							"parameter_mmax" : 1000.0,
							"parameter_modmode" : 0,
							"parameter_shortname" : "2",
							"parameter_type" : 0,
							"parameter_unitstyle" : 0
						}

					}

				}

			}
, 			{
				"box" : 				{
					"fontsize" : 10.0,
					"id" : "obj-52",
					"linecount" : 4,
					"maxclass" : "newobj",
					"numinlets" : 6,
					"numoutlets" : 4,
					"outlettype" : [ "float", "float", "float", "float" ],
					"patching_rect" : [ 2091.525473594665527, 1251.928559184074402, 262.711870670318604, 53.0 ],
					"text" : "jk.pushrotaryAlt 4 @dial1 r @dial2 b @init 500 @needle y @dialtextcolor k @bton y @btoff b @oncolor y @offcolor b @texton \"3\" @textoff \"Rotary 3\" @lo 0 @hi 1000 @inc 1"
				}

			}
, 			{
				"box" : 				{
					"activedialcolor" : [ 1.0, 0.0, 0.0, 1.0 ],
					"activefgdialcolor" : [ 0.0, 0.0, 1.0, 1.0 ],
					"activeneedlecolor" : [ 1.0, 1.0, 0.0, 1.0 ],
					"fgdialcolor" : [ 0.243137254901961, 0.698039215686274, 0.415686274509804, 1.0 ],
					"focusbordercolor" : [ 0.65098, 0.666667, 0.662745, 1.0 ],
					"fontname" : "Arial",
					"fontsize" : 10.0,
					"id" : "obj-47",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1995.714333295822144, 1314.285745620727539, 27.0, 48.0 ],
					"saved_attribute_attributes" : 					{
						"activedialcolor" : 						{
							"expression" : ""
						}
,
						"activefgdialcolor" : 						{
							"expression" : ""
						}
,
						"activeneedlecolor" : 						{
							"expression" : ""
						}
,
						"fgdialcolor" : 						{
							"expression" : ""
						}
,
						"focusbordercolor" : 						{
							"expression" : ""
						}
,
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "2[3]",
							"parameter_mmax" : 1000.0,
							"parameter_modmode" : 0,
							"parameter_shortname" : "2",
							"parameter_type" : 0,
							"parameter_unitstyle" : 0
						}

					}
,
					"varname" : "live.dial[28]"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-46",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 2189.024442434310913, 1102.439050674438477, 29.5, 22.0 ],
					"text" : "4"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-44",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 2187.804930210113525, 1021.951243877410889, 29.5, 22.0 ],
					"text" : "3"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-42",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 2190.434710800647736, 938.260838806629181, 29.5, 22.0 ],
					"text" : "2"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-40",
					"maxclass" : "newobj",
					"numinlets" : 0,
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
						"rect" : [ 31.0, 55.0, 1210.0, 739.0 ],
						"gridsize" : [ 15.0, 15.0 ],
						"boxes" : [ 							{
								"box" : 								{
									"id" : "obj-31",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 2,
									"outlettype" : [ "bang", "" ],
									"patching_rect" : [ 1022.0, 776.0, 34.0, 22.0 ],
									"text" : "sel 1"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-30",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 2,
									"outlettype" : [ "bang", "" ],
									"patching_rect" : [ 859.0, 532.0, 34.0, 22.0 ],
									"text" : "sel 1"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-29",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 2,
									"outlettype" : [ "bang", "" ],
									"patching_rect" : [ 801.25, 414.7613525390625, 34.0, 22.0 ],
									"text" : "sel 1"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-28",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 2,
									"outlettype" : [ "bang", "" ],
									"patching_rect" : [ 774.0, 314.0, 34.0, 22.0 ],
									"text" : "sel 1"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-24",
									"maxclass" : "message",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 797.0, 269.0, 50.0, 22.0 ],
									"text" : "0"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-22",
									"maxclass" : "message",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 981.0, 706.0, 29.5, 22.0 ],
									"text" : "3"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-20",
									"maxclass" : "message",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 864.0, 473.0, 29.5, 22.0 ],
									"text" : "2"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-18",
									"maxclass" : "message",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 830.0, 389.0, 29.5, 22.0 ],
									"text" : "1"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-16",
									"maxclass" : "message",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 765.225794434547424, 347.7613525390625, 29.5, 22.0 ],
									"text" : "0"
								}

							}
, 							{
								"box" : 								{
									"comment" : "",
									"id" : "obj-7",
									"index" : 1,
									"maxclass" : "outlet",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 866.0, 318.0, 30.0, 30.0 ]
								}

							}
, 							{
								"box" : 								{
									"activebgcolor" : [ 1.0, 1.0, 0.0, 1.0 ],
									"activebgoncolor" : [ 1.0, 0.0, 0.0, 1.0 ],
									"bordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
									"focusbordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
									"fontname" : "Arial",
									"fontsize" : 10.0,
									"id" : "obj-197",
									"maxclass" : "live.text",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "", "" ],
									"parameter_enable" : 1,
									"patching_rect" : [ 655.162622213363647, 913.196147084236145, 76.266666666666666, 34.0 ],
									"saved_attribute_attributes" : 									{
										"activebgcolor" : 										{
											"expression" : ""
										}
,
										"activebgoncolor" : 										{
											"expression" : ""
										}
,
										"activetextcolor" : 										{
											"expression" : ""
										}
,
										"activetextoncolor" : 										{
											"expression" : ""
										}
,
										"bordercolor" : 										{
											"expression" : ""
										}
,
										"focusbordercolor" : 										{
											"expression" : ""
										}
,
										"valueof" : 										{
											"parameter_enum" : [ "val1", "val2" ],
											"parameter_longname" : "live.text[31]",
											"parameter_mmax" : 1,
											"parameter_modmode" : 0,
											"parameter_shortname" : "live.text",
											"parameter_type" : 2
										}

									}
,
									"text" : "bippity",
									"texton" : "boppity",
									"varname" : "live.text[29]"
								}

							}
, 							{
								"box" : 								{
									"fontsize" : 10.0,
									"id" : "obj-198",
									"linecount" : 2,
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 4,
									"outlettype" : [ "int", "", "", "" ],
									"patching_rect" : [ 655.162622213363647, 868.26861047744751, 307.0, 31.0 ],
									"text" : "jk.push @pad 15 @oncolor r @offcolor y @texton boppity @textoff bippity @mode b"
								}

							}
, 							{
								"box" : 								{
									"activebgcolor" : [ 1.0, 1.0, 0.0, 1.0 ],
									"activebgoncolor" : [ 1.0, 0.0, 0.0, 1.0 ],
									"bordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
									"focusbordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
									"fontname" : "Arial",
									"fontsize" : 10.0,
									"id" : "obj-199",
									"maxclass" : "live.text",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "", "" ],
									"parameter_enable" : 1,
									"patching_rect" : [ 462.408997416496277, 913.196147084236145, 76.266666666666666, 34.0 ],
									"saved_attribute_attributes" : 									{
										"activebgcolor" : 										{
											"expression" : ""
										}
,
										"activebgoncolor" : 										{
											"expression" : ""
										}
,
										"activetextcolor" : 										{
											"expression" : ""
										}
,
										"activetextoncolor" : 										{
											"expression" : ""
										}
,
										"bordercolor" : 										{
											"expression" : ""
										}
,
										"focusbordercolor" : 										{
											"expression" : ""
										}
,
										"valueof" : 										{
											"parameter_enum" : [ "val1", "val2" ],
											"parameter_longname" : "live.text[32]",
											"parameter_mmax" : 1,
											"parameter_modmode" : 0,
											"parameter_shortname" : "live.text",
											"parameter_type" : 2
										}

									}
,
									"text" : "bippity",
									"texton" : "boppity",
									"varname" : "live.text[30]"
								}

							}
, 							{
								"box" : 								{
									"fontsize" : 10.0,
									"id" : "obj-200",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 4,
									"outlettype" : [ "int", "", "", "" ],
									"patching_rect" : [ 462.408997416496277, 868.26861047744751, 382.0, 20.0 ],
									"text" : "jk.push @pad 14 @oncolor r @offcolor y @texton boppity @textoff bippity @mode b"
								}

							}
, 							{
								"box" : 								{
									"activebgcolor" : [ 1.0, 1.0, 0.0, 1.0 ],
									"activebgoncolor" : [ 1.0, 0.0, 0.0, 1.0 ],
									"bordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
									"focusbordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
									"fontname" : "Arial",
									"fontsize" : 10.0,
									"id" : "obj-201",
									"maxclass" : "live.text",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "", "" ],
									"parameter_enable" : 1,
									"patching_rect" : [ 263.85827112197876, 917.543973207473755, 76.266666666666666, 34.0 ],
									"saved_attribute_attributes" : 									{
										"activebgcolor" : 										{
											"expression" : ""
										}
,
										"activebgoncolor" : 										{
											"expression" : ""
										}
,
										"activetextcolor" : 										{
											"expression" : ""
										}
,
										"activetextoncolor" : 										{
											"expression" : ""
										}
,
										"bordercolor" : 										{
											"expression" : ""
										}
,
										"focusbordercolor" : 										{
											"expression" : ""
										}
,
										"valueof" : 										{
											"parameter_enum" : [ "val1", "val2" ],
											"parameter_longname" : "live.text[33]",
											"parameter_mmax" : 1,
											"parameter_modmode" : 0,
											"parameter_shortname" : "live.text",
											"parameter_type" : 2
										}

									}
,
									"text" : "bippity",
									"texton" : "boppity",
									"varname" : "live.text[31]"
								}

							}
, 							{
								"box" : 								{
									"fontsize" : 10.0,
									"id" : "obj-202",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 4,
									"outlettype" : [ "int", "", "", "" ],
									"patching_rect" : [ 263.85827112197876, 872.61643660068512, 382.0, 20.0 ],
									"text" : "jk.push @pad 13 @oncolor r @offcolor y @texton boppity @textoff bippity @mode b"
								}

							}
, 							{
								"box" : 								{
									"activebgcolor" : [ 1.0, 1.0, 0.0, 1.0 ],
									"activebgoncolor" : [ 1.0, 0.0, 0.0, 1.0 ],
									"bordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
									"focusbordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
									"fontname" : "Arial",
									"fontsize" : 10.0,
									"id" : "obj-203",
									"maxclass" : "live.text",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "", "" ],
									"parameter_enable" : 1,
									"patching_rect" : [ 69.655370950698853, 917.543973207473755, 76.266666666666666, 34.0 ],
									"saved_attribute_attributes" : 									{
										"activebgcolor" : 										{
											"expression" : ""
										}
,
										"activebgoncolor" : 										{
											"expression" : ""
										}
,
										"activetextcolor" : 										{
											"expression" : ""
										}
,
										"activetextoncolor" : 										{
											"expression" : ""
										}
,
										"bordercolor" : 										{
											"expression" : ""
										}
,
										"focusbordercolor" : 										{
											"expression" : ""
										}
,
										"valueof" : 										{
											"parameter_enum" : [ "val1", "val2" ],
											"parameter_longname" : "live.text[34]",
											"parameter_mmax" : 1,
											"parameter_modmode" : 0,
											"parameter_shortname" : "live.text",
											"parameter_type" : 2
										}

									}
,
									"text" : "bippity",
									"texton" : "boppity",
									"varname" : "live.text[32]"
								}

							}
, 							{
								"box" : 								{
									"fontsize" : 10.0,
									"id" : "obj-204",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 4,
									"outlettype" : [ "int", "", "", "" ],
									"patching_rect" : [ 69.655370950698853, 872.61643660068512, 382.0, 20.0 ],
									"text" : "jk.push @pad 12 @oncolor r @offcolor y @texton boppity @textoff bippity @mode b"
								}

							}
, 							{
								"box" : 								{
									"activebgcolor" : [ 1.0, 1.0, 0.0, 1.0 ],
									"activebgoncolor" : [ 1.0, 0.0, 0.0, 1.0 ],
									"bordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
									"focusbordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
									"fontname" : "Arial",
									"fontsize" : 10.0,
									"id" : "obj-193",
									"maxclass" : "live.text",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "", "" ],
									"parameter_enable" : 1,
									"patching_rect" : [ 655.903403655687725, 818.630892276763916, 76.266666666666666, 34.0 ],
									"saved_attribute_attributes" : 									{
										"activebgcolor" : 										{
											"expression" : ""
										}
,
										"activebgoncolor" : 										{
											"expression" : ""
										}
,
										"activetextcolor" : 										{
											"expression" : ""
										}
,
										"activetextoncolor" : 										{
											"expression" : ""
										}
,
										"bordercolor" : 										{
											"expression" : ""
										}
,
										"focusbordercolor" : 										{
											"expression" : ""
										}
,
										"valueof" : 										{
											"parameter_enum" : [ "val1", "val2" ],
											"parameter_longname" : "live.text[29]",
											"parameter_mmax" : 1,
											"parameter_modmode" : 0,
											"parameter_shortname" : "live.text",
											"parameter_type" : 2
										}

									}
,
									"text" : "bippity",
									"texton" : "boppity",
									"varname" : "live.text[27]"
								}

							}
, 							{
								"box" : 								{
									"fontsize" : 10.0,
									"id" : "obj-194",
									"linecount" : 2,
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 4,
									"outlettype" : [ "int", "", "", "" ],
									"patching_rect" : [ 655.903403655687725, 772.978719234466553, 306.0, 31.0 ],
									"text" : "jk.push @pad 11 @oncolor r @offcolor y @texton boppity @textoff bippity @mode b"
								}

							}
, 							{
								"box" : 								{
									"activebgcolor" : [ 1.0, 1.0, 0.0, 1.0 ],
									"activebgoncolor" : [ 1.0, 0.0, 0.0, 1.0 ],
									"bordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
									"focusbordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
									"fontname" : "Arial",
									"fontsize" : 10.0,
									"id" : "obj-195",
									"maxclass" : "live.text",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "", "" ],
									"parameter_enable" : 1,
									"patching_rect" : [ 462.425146476427471, 818.630892276763916, 76.266666666666666, 34.0 ],
									"saved_attribute_attributes" : 									{
										"activebgcolor" : 										{
											"expression" : ""
										}
,
										"activebgoncolor" : 										{
											"expression" : ""
										}
,
										"activetextcolor" : 										{
											"expression" : ""
										}
,
										"activetextoncolor" : 										{
											"expression" : ""
										}
,
										"bordercolor" : 										{
											"expression" : ""
										}
,
										"focusbordercolor" : 										{
											"expression" : ""
										}
,
										"valueof" : 										{
											"parameter_enum" : [ "val1", "val2" ],
											"parameter_longname" : "live.text[30]",
											"parameter_mmax" : 1,
											"parameter_modmode" : 0,
											"parameter_shortname" : "live.text",
											"parameter_type" : 2
										}

									}
,
									"text" : "bippity",
									"texton" : "boppity",
									"varname" : "live.text[28]"
								}

							}
, 							{
								"box" : 								{
									"fontsize" : 10.0,
									"id" : "obj-196",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 4,
									"outlettype" : [ "int", "", "", "" ],
									"patching_rect" : [ 462.425146476427471, 772.978719234466553, 382.0, 20.0 ],
									"text" : "jk.push @pad 10 @oncolor r @offcolor y @texton boppity @textoff bippity @mode b"
								}

							}
, 							{
								"box" : 								{
									"activebgcolor" : [ 1.0, 1.0, 0.0, 1.0 ],
									"activebgoncolor" : [ 1.0, 0.0, 0.0, 1.0 ],
									"bordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
									"focusbordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
									"fontname" : "Arial",
									"fontsize" : 10.0,
									"id" : "obj-191",
									"maxclass" : "live.text",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "", "" ],
									"parameter_enable" : 1,
									"patching_rect" : [ 263.85827112197876, 822.699227333068848, 76.266666666666666, 34.0 ],
									"saved_attribute_attributes" : 									{
										"activebgcolor" : 										{
											"expression" : ""
										}
,
										"activebgoncolor" : 										{
											"expression" : ""
										}
,
										"activetextcolor" : 										{
											"expression" : ""
										}
,
										"activetextoncolor" : 										{
											"expression" : ""
										}
,
										"bordercolor" : 										{
											"expression" : ""
										}
,
										"focusbordercolor" : 										{
											"expression" : ""
										}
,
										"valueof" : 										{
											"parameter_enum" : [ "val1", "val2" ],
											"parameter_longname" : "live.text[28]",
											"parameter_mmax" : 1,
											"parameter_modmode" : 0,
											"parameter_shortname" : "live.text",
											"parameter_type" : 2
										}

									}
,
									"text" : "bippity",
									"texton" : "boppity",
									"varname" : "live.text[26]"
								}

							}
, 							{
								"box" : 								{
									"fontsize" : 10.0,
									"id" : "obj-192",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 4,
									"outlettype" : [ "int", "", "", "" ],
									"patching_rect" : [ 263.85827112197876, 777.047054290771484, 376.0, 20.0 ],
									"text" : "jk.push @pad 9 @oncolor r @offcolor y @texton boppity @textoff bippity @mode b"
								}

							}
, 							{
								"box" : 								{
									"activebgcolor" : [ 1.0, 1.0, 0.0, 1.0 ],
									"activebgoncolor" : [ 1.0, 0.0, 0.0, 1.0 ],
									"bordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
									"focusbordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
									"fontname" : "Arial",
									"fontsize" : 10.0,
									"id" : "obj-189",
									"maxclass" : "live.text",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "", "" ],
									"parameter_enable" : 1,
									"patching_rect" : [ 70.274650871753693, 823.047054290771484, 76.266666666666666, 34.0 ],
									"saved_attribute_attributes" : 									{
										"activebgcolor" : 										{
											"expression" : ""
										}
,
										"activebgoncolor" : 										{
											"expression" : ""
										}
,
										"activetextcolor" : 										{
											"expression" : ""
										}
,
										"activetextoncolor" : 										{
											"expression" : ""
										}
,
										"bordercolor" : 										{
											"expression" : ""
										}
,
										"focusbordercolor" : 										{
											"expression" : ""
										}
,
										"valueof" : 										{
											"parameter_enum" : [ "val1", "val2" ],
											"parameter_longname" : "live.text[24]",
											"parameter_mmax" : 1,
											"parameter_modmode" : 0,
											"parameter_shortname" : "live.text",
											"parameter_type" : 2
										}

									}
,
									"text" : "bippity",
									"texton" : "boppity",
									"varname" : "live.text[25]"
								}

							}
, 							{
								"box" : 								{
									"fontsize" : 10.0,
									"id" : "obj-190",
									"linecount" : 3,
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 4,
									"outlettype" : [ "int", "", "", "" ],
									"patching_rect" : [ 70.380013942718506, 777.047054290771484, 159.782605648040771, 42.0 ],
									"text" : "jk.push @pad 8 @oncolor r @offcolor y @texton boppity @textoff bippity @mode b"
								}

							}
, 							{
								"box" : 								{
									"activebgcolor" : [ 0.8, 0.35, 0.95, 1.0 ],
									"activebgoncolor" : [ 1.0, 0.0, 0.0, 1.0 ],
									"activetextcolor" : [ 1.0, 1.0, 1.0, 1.0 ],
									"bordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
									"focusbordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
									"fontname" : "Arial",
									"fontsize" : 10.0,
									"id" : "obj-165",
									"maxclass" : "live.text",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "", "" ],
									"parameter_enable" : 1,
									"patching_rect" : [ 679.0322265625, 694.7613525390625, 76.266666666666666, 34.0 ],
									"saved_attribute_attributes" : 									{
										"activebgcolor" : 										{
											"expression" : ""
										}
,
										"activebgoncolor" : 										{
											"expression" : ""
										}
,
										"activetextcolor" : 										{
											"expression" : ""
										}
,
										"activetextoncolor" : 										{
											"expression" : ""
										}
,
										"bordercolor" : 										{
											"expression" : ""
										}
,
										"focusbordercolor" : 										{
											"expression" : ""
										}
,
										"valueof" : 										{
											"parameter_enum" : [ "val1", "val2" ],
											"parameter_longname" : "live.text[17]",
											"parameter_mmax" : 1,
											"parameter_modmode" : 0,
											"parameter_shortname" : "live.text",
											"parameter_type" : 2
										}

									}
,
									"text" : "bippity",
									"texton" : "boppity",
									"varname" : "live.text[21]"
								}

							}
, 							{
								"box" : 								{
									"fontsize" : 10.0,
									"id" : "obj-166",
									"linecount" : 3,
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 4,
									"outlettype" : [ "int", "", "", "" ],
									"patching_rect" : [ 679.0322265625, 648.7613525390625, 151.515138149261475, 42.0 ],
									"text" : "jk.push @pad 31 @oncolor r @offcolor p @texton boppity @textoff bippity @mode b"
								}

							}
, 							{
								"box" : 								{
									"activebgcolor" : [ 0.8, 0.35, 0.95, 1.0 ],
									"activebgoncolor" : [ 1.0, 0.0, 0.0, 1.0 ],
									"activetextcolor" : [ 1.0, 1.0, 1.0, 1.0 ],
									"bordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
									"focusbordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
									"fontname" : "Arial",
									"fontsize" : 10.0,
									"id" : "obj-167",
									"maxclass" : "live.text",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "", "" ],
									"parameter_enable" : 1,
									"patching_rect" : [ 473.0322265625, 688.7613525390625, 76.266666666666666, 34.0 ],
									"saved_attribute_attributes" : 									{
										"activebgcolor" : 										{
											"expression" : ""
										}
,
										"activebgoncolor" : 										{
											"expression" : ""
										}
,
										"activetextcolor" : 										{
											"expression" : ""
										}
,
										"activetextoncolor" : 										{
											"expression" : ""
										}
,
										"bordercolor" : 										{
											"expression" : ""
										}
,
										"focusbordercolor" : 										{
											"expression" : ""
										}
,
										"valueof" : 										{
											"parameter_enum" : [ "val1", "val2" ],
											"parameter_longname" : "live.text[18]",
											"parameter_mmax" : 1,
											"parameter_modmode" : 0,
											"parameter_shortname" : "live.text",
											"parameter_type" : 2
										}

									}
,
									"text" : "bippity",
									"texton" : "boppity",
									"varname" : "live.text[22]"
								}

							}
, 							{
								"box" : 								{
									"fontsize" : 10.0,
									"id" : "obj-168",
									"linecount" : 3,
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 4,
									"outlettype" : [ "int", "", "", "" ],
									"patching_rect" : [ 471.0322265625, 644.7613525390625, 151.515138149261475, 42.0 ],
									"text" : "jk.push @pad 30 @oncolor r @offcolor p @texton boppity @textoff bippity @mode b"
								}

							}
, 							{
								"box" : 								{
									"activebgcolor" : [ 0.8, 0.35, 0.95, 1.0 ],
									"activebgoncolor" : [ 1.0, 0.0, 0.0, 1.0 ],
									"activetextcolor" : [ 1.0, 1.0, 1.0, 1.0 ],
									"bordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
									"focusbordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
									"fontname" : "Arial",
									"fontsize" : 10.0,
									"id" : "obj-169",
									"maxclass" : "live.text",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "", "" ],
									"parameter_enable" : 1,
									"patching_rect" : [ 281.0322265625, 690.7613525390625, 76.266666666666666, 34.0 ],
									"saved_attribute_attributes" : 									{
										"activebgcolor" : 										{
											"expression" : ""
										}
,
										"activebgoncolor" : 										{
											"expression" : ""
										}
,
										"activetextcolor" : 										{
											"expression" : ""
										}
,
										"activetextoncolor" : 										{
											"expression" : ""
										}
,
										"bordercolor" : 										{
											"expression" : ""
										}
,
										"focusbordercolor" : 										{
											"expression" : ""
										}
,
										"valueof" : 										{
											"parameter_enum" : [ "val1", "val2" ],
											"parameter_longname" : "live.text[19]",
											"parameter_mmax" : 1,
											"parameter_modmode" : 0,
											"parameter_shortname" : "live.text",
											"parameter_type" : 2
										}

									}
,
									"text" : "bippity",
									"texton" : "boppity",
									"varname" : "live.text[23]"
								}

							}
, 							{
								"box" : 								{
									"fontsize" : 10.0,
									"id" : "obj-170",
									"linecount" : 3,
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 4,
									"outlettype" : [ "int", "", "", "" ],
									"patching_rect" : [ 281.0322265625, 644.7613525390625, 151.515138149261475, 42.0 ],
									"text" : "jk.push @pad 29 @oncolor r @offcolor p @texton boppity @textoff bippity @mode b"
								}

							}
, 							{
								"box" : 								{
									"activebgcolor" : [ 0.8, 0.35, 0.95, 1.0 ],
									"activebgoncolor" : [ 1.0, 0.0, 0.0, 1.0 ],
									"activetextcolor" : [ 1.0, 1.0, 1.0, 1.0 ],
									"bordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
									"focusbordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
									"fontname" : "Arial",
									"fontsize" : 10.0,
									"id" : "obj-171",
									"maxclass" : "live.text",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "", "" ],
									"parameter_enable" : 1,
									"patching_rect" : [ 73.0322265625, 690.7613525390625, 76.266666666666666, 34.0 ],
									"saved_attribute_attributes" : 									{
										"activebgcolor" : 										{
											"expression" : ""
										}
,
										"activebgoncolor" : 										{
											"expression" : ""
										}
,
										"activetextcolor" : 										{
											"expression" : ""
										}
,
										"activetextoncolor" : 										{
											"expression" : ""
										}
,
										"bordercolor" : 										{
											"expression" : ""
										}
,
										"focusbordercolor" : 										{
											"expression" : ""
										}
,
										"valueof" : 										{
											"parameter_enum" : [ "val1", "val2" ],
											"parameter_longname" : "live.text[20]",
											"parameter_mmax" : 1,
											"parameter_modmode" : 0,
											"parameter_shortname" : "live.text",
											"parameter_type" : 2
										}

									}
,
									"text" : "bippity",
									"texton" : "boppity",
									"varname" : "live.text[24]"
								}

							}
, 							{
								"box" : 								{
									"fontsize" : 10.0,
									"id" : "obj-172",
									"linecount" : 3,
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 4,
									"outlettype" : [ "int", "", "", "" ],
									"patching_rect" : [ 71.0322265625, 644.7613525390625, 151.515138149261475, 42.0 ],
									"text" : "jk.push @pad 28 @oncolor r @offcolor p @texton boppity @textoff bippity @mode b"
								}

							}
, 							{
								"box" : 								{
									"activebgcolor" : [ 0.8, 0.35, 0.95, 1.0 ],
									"activebgoncolor" : [ 1.0, 0.0, 0.0, 1.0 ],
									"activetextcolor" : [ 1.0, 1.0, 1.0, 1.0 ],
									"bordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
									"focusbordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
									"fontname" : "Arial",
									"fontsize" : 10.0,
									"id" : "obj-163",
									"maxclass" : "live.text",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "", "" ],
									"parameter_enable" : 1,
									"patching_rect" : [ 679.0322265625, 578.7613525390625, 76.266666666666666, 34.0 ],
									"saved_attribute_attributes" : 									{
										"activebgcolor" : 										{
											"expression" : ""
										}
,
										"activebgoncolor" : 										{
											"expression" : ""
										}
,
										"activetextcolor" : 										{
											"expression" : ""
										}
,
										"activetextoncolor" : 										{
											"expression" : ""
										}
,
										"bordercolor" : 										{
											"expression" : ""
										}
,
										"focusbordercolor" : 										{
											"expression" : ""
										}
,
										"valueof" : 										{
											"parameter_enum" : [ "val1", "val2" ],
											"parameter_longname" : "live.text[16]",
											"parameter_mmax" : 1,
											"parameter_modmode" : 0,
											"parameter_shortname" : "live.text",
											"parameter_type" : 2
										}

									}
,
									"text" : "bippity",
									"texton" : "boppity",
									"varname" : "live.text[20]"
								}

							}
, 							{
								"box" : 								{
									"fontsize" : 10.0,
									"id" : "obj-164",
									"linecount" : 3,
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 4,
									"outlettype" : [ "int", "", "", "" ],
									"patching_rect" : [ 679.0322265625, 532.7613525390625, 151.515138149261475, 42.0 ],
									"text" : "jk.push @pad 27 @oncolor r @offcolor p @texton boppity @textoff bippity @mode b"
								}

							}
, 							{
								"box" : 								{
									"activebgcolor" : [ 0.8, 0.35, 0.95, 1.0 ],
									"activebgoncolor" : [ 1.0, 0.0, 0.0, 1.0 ],
									"activetextcolor" : [ 1.0, 1.0, 1.0, 1.0 ],
									"bordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
									"focusbordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
									"fontname" : "Arial",
									"fontsize" : 10.0,
									"id" : "obj-161",
									"maxclass" : "live.text",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "", "" ],
									"parameter_enable" : 1,
									"patching_rect" : [ 475.0322265625, 574.7613525390625, 76.266666666666666, 34.0 ],
									"saved_attribute_attributes" : 									{
										"activebgcolor" : 										{
											"expression" : ""
										}
,
										"activebgoncolor" : 										{
											"expression" : ""
										}
,
										"activetextcolor" : 										{
											"expression" : ""
										}
,
										"activetextoncolor" : 										{
											"expression" : ""
										}
,
										"bordercolor" : 										{
											"expression" : ""
										}
,
										"focusbordercolor" : 										{
											"expression" : ""
										}
,
										"valueof" : 										{
											"parameter_enum" : [ "val1", "val2" ],
											"parameter_longname" : "live.text[15]",
											"parameter_mmax" : 1,
											"parameter_modmode" : 0,
											"parameter_shortname" : "live.text",
											"parameter_type" : 2
										}

									}
,
									"text" : "bippity",
									"texton" : "boppity",
									"varname" : "live.text[19]"
								}

							}
, 							{
								"box" : 								{
									"fontsize" : 10.0,
									"id" : "obj-162",
									"linecount" : 3,
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 4,
									"outlettype" : [ "int", "", "", "" ],
									"patching_rect" : [ 473.0322265625, 530.7613525390625, 151.515138149261475, 42.0 ],
									"text" : "jk.push @pad 26 @oncolor r @offcolor p @texton boppity @textoff bippity @mode b"
								}

							}
, 							{
								"box" : 								{
									"activebgcolor" : [ 0.8, 0.35, 0.95, 1.0 ],
									"activebgoncolor" : [ 1.0, 0.0, 0.0, 1.0 ],
									"activetextcolor" : [ 1.0, 1.0, 1.0, 1.0 ],
									"bordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
									"focusbordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
									"fontname" : "Arial",
									"fontsize" : 10.0,
									"id" : "obj-159",
									"maxclass" : "live.text",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "", "" ],
									"parameter_enable" : 1,
									"patching_rect" : [ 283.0322265625, 576.7613525390625, 76.266666666666666, 34.0 ],
									"saved_attribute_attributes" : 									{
										"activebgcolor" : 										{
											"expression" : ""
										}
,
										"activebgoncolor" : 										{
											"expression" : ""
										}
,
										"activetextcolor" : 										{
											"expression" : ""
										}
,
										"activetextoncolor" : 										{
											"expression" : ""
										}
,
										"bordercolor" : 										{
											"expression" : ""
										}
,
										"focusbordercolor" : 										{
											"expression" : ""
										}
,
										"valueof" : 										{
											"parameter_enum" : [ "val1", "val2" ],
											"parameter_longname" : "live.text[14]",
											"parameter_mmax" : 1,
											"parameter_modmode" : 0,
											"parameter_shortname" : "live.text",
											"parameter_type" : 2
										}

									}
,
									"text" : "bippity",
									"texton" : "boppity",
									"varname" : "live.text[18]"
								}

							}
, 							{
								"box" : 								{
									"fontsize" : 10.0,
									"id" : "obj-160",
									"linecount" : 3,
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 4,
									"outlettype" : [ "int", "", "", "" ],
									"patching_rect" : [ 281.0322265625, 530.7613525390625, 151.515138149261475, 42.0 ],
									"text" : "jk.push @pad 25 @oncolor r @offcolor p @texton boppity @textoff bippity @mode b"
								}

							}
, 							{
								"box" : 								{
									"activebgcolor" : [ 0.8, 0.35, 0.95, 1.0 ],
									"activebgoncolor" : [ 1.0, 0.0, 0.0, 1.0 ],
									"activetextcolor" : [ 1.0, 1.0, 1.0, 1.0 ],
									"bordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
									"focusbordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
									"fontname" : "Arial",
									"fontsize" : 10.0,
									"id" : "obj-157",
									"maxclass" : "live.text",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "", "" ],
									"parameter_enable" : 1,
									"patching_rect" : [ 73.0322265625, 576.7613525390625, 76.266666666666666, 34.0 ],
									"saved_attribute_attributes" : 									{
										"activebgcolor" : 										{
											"expression" : ""
										}
,
										"activebgoncolor" : 										{
											"expression" : ""
										}
,
										"activetextcolor" : 										{
											"expression" : ""
										}
,
										"activetextoncolor" : 										{
											"expression" : ""
										}
,
										"bordercolor" : 										{
											"expression" : ""
										}
,
										"focusbordercolor" : 										{
											"expression" : ""
										}
,
										"valueof" : 										{
											"parameter_enum" : [ "val1", "val2" ],
											"parameter_longname" : "live.text[13]",
											"parameter_mmax" : 1,
											"parameter_modmode" : 0,
											"parameter_shortname" : "live.text",
											"parameter_type" : 2
										}

									}
,
									"text" : "bippity",
									"texton" : "boppity",
									"varname" : "live.text[15]"
								}

							}
, 							{
								"box" : 								{
									"fontsize" : 10.0,
									"id" : "obj-158",
									"linecount" : 3,
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 4,
									"outlettype" : [ "int", "", "", "" ],
									"patching_rect" : [ 73.0322265625, 530.7613525390625, 153.030289530754089, 42.0 ],
									"text" : "jk.push @pad 24 @oncolor r @offcolor p @texton boppity @textoff bippity @mode b"
								}

							}
, 							{
								"box" : 								{
									"activebgcolor" : [ 0.0, 1.0, 0.0, 1.0 ],
									"activebgoncolor" : [ 1.0, 0.0, 0.0, 1.0 ],
									"bordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
									"focusbordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
									"fontname" : "Arial",
									"fontsize" : 10.0,
									"id" : "obj-155",
									"maxclass" : "live.text",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "", "" ],
									"parameter_enable" : 1,
									"patching_rect" : [ 575.612889409065247, 475.7613525390625, 76.266666666666666, 34.0 ],
									"saved_attribute_attributes" : 									{
										"activebgcolor" : 										{
											"expression" : ""
										}
,
										"activebgoncolor" : 										{
											"expression" : ""
										}
,
										"activetextcolor" : 										{
											"expression" : ""
										}
,
										"activetextoncolor" : 										{
											"expression" : ""
										}
,
										"bordercolor" : 										{
											"expression" : ""
										}
,
										"focusbordercolor" : 										{
											"expression" : ""
										}
,
										"valueof" : 										{
											"parameter_enum" : [ "val1", "val2" ],
											"parameter_longname" : "live.text[12]",
											"parameter_mmax" : 1,
											"parameter_modmode" : 0,
											"parameter_shortname" : "live.text",
											"parameter_type" : 2
										}

									}
,
									"text" : "bippity",
									"texton" : "boppity",
									"varname" : "live.text[14]"
								}

							}
, 							{
								"box" : 								{
									"fontsize" : 10.0,
									"id" : "obj-156",
									"linecount" : 3,
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 4,
									"outlettype" : [ "int", "", "", "" ],
									"patching_rect" : [ 576.0322265625, 430.7613525390625, 158.0, 42.0 ],
									"text" : "jk.push @pad 47 @oncolor r @offcolor g @texton boppity @textoff bippity @mode b"
								}

							}
, 							{
								"box" : 								{
									"activebgcolor" : [ 0.0, 1.0, 0.0, 1.0 ],
									"activebgoncolor" : [ 1.0, 0.0, 0.0, 1.0 ],
									"bordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
									"focusbordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
									"fontname" : "Arial",
									"fontsize" : 10.0,
									"id" : "obj-153",
									"maxclass" : "live.text",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "", "" ],
									"parameter_enable" : 1,
									"patching_rect" : [ 413.0322265625, 479.7613525390625, 76.266666666666666, 34.0 ],
									"saved_attribute_attributes" : 									{
										"activebgcolor" : 										{
											"expression" : ""
										}
,
										"activebgoncolor" : 										{
											"expression" : ""
										}
,
										"activetextcolor" : 										{
											"expression" : ""
										}
,
										"activetextoncolor" : 										{
											"expression" : ""
										}
,
										"bordercolor" : 										{
											"expression" : ""
										}
,
										"focusbordercolor" : 										{
											"expression" : ""
										}
,
										"valueof" : 										{
											"parameter_enum" : [ "val1", "val2" ],
											"parameter_longname" : "live.text[11]",
											"parameter_mmax" : 1,
											"parameter_modmode" : 0,
											"parameter_shortname" : "live.text",
											"parameter_type" : 2
										}

									}
,
									"text" : "bippity",
									"texton" : "boppity",
									"varname" : "live.text[13]"
								}

							}
, 							{
								"box" : 								{
									"fontsize" : 10.0,
									"id" : "obj-154",
									"linecount" : 3,
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 4,
									"outlettype" : [ "int", "", "", "" ],
									"patching_rect" : [ 413.0322265625, 434.7613525390625, 158.0, 42.0 ],
									"text" : "jk.push @pad 46 @oncolor r @offcolor g @texton boppity @textoff bippity @mode b"
								}

							}
, 							{
								"box" : 								{
									"activebgcolor" : [ 0.0, 1.0, 0.0, 1.0 ],
									"activebgoncolor" : [ 1.0, 0.0, 0.0, 1.0 ],
									"bordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
									"focusbordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
									"fontname" : "Arial",
									"fontsize" : 10.0,
									"id" : "obj-151",
									"maxclass" : "live.text",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "", "" ],
									"parameter_enable" : 1,
									"patching_rect" : [ 250.0322265625, 479.7613525390625, 76.266666666666666, 34.0 ],
									"saved_attribute_attributes" : 									{
										"activebgcolor" : 										{
											"expression" : ""
										}
,
										"activebgoncolor" : 										{
											"expression" : ""
										}
,
										"activetextcolor" : 										{
											"expression" : ""
										}
,
										"activetextoncolor" : 										{
											"expression" : ""
										}
,
										"bordercolor" : 										{
											"expression" : ""
										}
,
										"focusbordercolor" : 										{
											"expression" : ""
										}
,
										"valueof" : 										{
											"parameter_enum" : [ "val1", "val2" ],
											"parameter_longname" : "live.text[10]",
											"parameter_mmax" : 1,
											"parameter_modmode" : 0,
											"parameter_shortname" : "live.text",
											"parameter_type" : 2
										}

									}
,
									"text" : "bippity",
									"texton" : "boppity",
									"varname" : "live.text[12]"
								}

							}
, 							{
								"box" : 								{
									"fontsize" : 10.0,
									"id" : "obj-152",
									"linecount" : 3,
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 4,
									"outlettype" : [ "int", "", "", "" ],
									"patching_rect" : [ 250.0322265625, 434.7613525390625, 158.0, 42.0 ],
									"text" : "jk.push @pad 45 @oncolor r @offcolor g @texton boppity @textoff bippity @mode b"
								}

							}
, 							{
								"box" : 								{
									"activebgcolor" : [ 0.0, 1.0, 0.0, 1.0 ],
									"activebgoncolor" : [ 1.0, 0.0, 0.0, 1.0 ],
									"bordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
									"focusbordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
									"fontname" : "Arial",
									"fontsize" : 10.0,
									"id" : "obj-149",
									"maxclass" : "live.text",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "", "" ],
									"parameter_enable" : 1,
									"patching_rect" : [ 81.0322265625, 479.7613525390625, 76.266666666666666, 34.0 ],
									"saved_attribute_attributes" : 									{
										"activebgcolor" : 										{
											"expression" : ""
										}
,
										"activebgoncolor" : 										{
											"expression" : ""
										}
,
										"activetextcolor" : 										{
											"expression" : ""
										}
,
										"activetextoncolor" : 										{
											"expression" : ""
										}
,
										"bordercolor" : 										{
											"expression" : ""
										}
,
										"focusbordercolor" : 										{
											"expression" : ""
										}
,
										"valueof" : 										{
											"parameter_enum" : [ "val1", "val2" ],
											"parameter_longname" : "live.text[9]",
											"parameter_mmax" : 1,
											"parameter_modmode" : 0,
											"parameter_shortname" : "live.text",
											"parameter_type" : 2
										}

									}
,
									"text" : "bippity",
									"texton" : "boppity",
									"varname" : "live.text[11]"
								}

							}
, 							{
								"box" : 								{
									"fontsize" : 10.0,
									"id" : "obj-150",
									"linecount" : 3,
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 4,
									"outlettype" : [ "int", "", "", "" ],
									"patching_rect" : [ 81.0322265625, 434.7613525390625, 158.0, 42.0 ],
									"text" : "jk.push @pad 44 @oncolor r @offcolor g @texton boppity @textoff bippity @mode b"
								}

							}
, 							{
								"box" : 								{
									"activebgcolor" : [ 0.0, 1.0, 0.0, 1.0 ],
									"activebgoncolor" : [ 1.0, 0.0, 0.0, 1.0 ],
									"bordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
									"focusbordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
									"fontname" : "Arial",
									"fontsize" : 10.0,
									"id" : "obj-147",
									"maxclass" : "live.text",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "", "" ],
									"parameter_enable" : 1,
									"patching_rect" : [ 575.612889409065247, 375.7613525390625, 76.266666666666666, 34.0 ],
									"saved_attribute_attributes" : 									{
										"activebgcolor" : 										{
											"expression" : ""
										}
,
										"activebgoncolor" : 										{
											"expression" : ""
										}
,
										"activetextcolor" : 										{
											"expression" : ""
										}
,
										"activetextoncolor" : 										{
											"expression" : ""
										}
,
										"bordercolor" : 										{
											"expression" : ""
										}
,
										"focusbordercolor" : 										{
											"expression" : ""
										}
,
										"valueof" : 										{
											"parameter_enum" : [ "val1", "val2" ],
											"parameter_longname" : "live.text[8]",
											"parameter_mmax" : 1,
											"parameter_modmode" : 0,
											"parameter_shortname" : "live.text",
											"parameter_type" : 2
										}

									}
,
									"text" : "bippity",
									"texton" : "boppity",
									"varname" : "live.text[10]"
								}

							}
, 							{
								"box" : 								{
									"fontsize" : 10.0,
									"id" : "obj-148",
									"linecount" : 3,
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 4,
									"outlettype" : [ "int", "", "", "" ],
									"patching_rect" : [ 576.0322265625, 329.7613525390625, 158.0, 42.0 ],
									"text" : "jk.push @pad 43 @oncolor r @offcolor g @texton boppity @textoff bippity @mode b"
								}

							}
, 							{
								"box" : 								{
									"activebgcolor" : [ 0.0, 1.0, 0.0, 1.0 ],
									"activebgoncolor" : [ 1.0, 0.0, 0.0, 1.0 ],
									"bordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
									"focusbordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
									"fontname" : "Arial",
									"fontsize" : 10.0,
									"id" : "obj-145",
									"maxclass" : "live.text",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "", "" ],
									"parameter_enable" : 1,
									"patching_rect" : [ 389.0322265625, 382.7613525390625, 76.266666666666666, 34.0 ],
									"saved_attribute_attributes" : 									{
										"activebgcolor" : 										{
											"expression" : ""
										}
,
										"activebgoncolor" : 										{
											"expression" : ""
										}
,
										"activetextcolor" : 										{
											"expression" : ""
										}
,
										"activetextoncolor" : 										{
											"expression" : ""
										}
,
										"bordercolor" : 										{
											"expression" : ""
										}
,
										"focusbordercolor" : 										{
											"expression" : ""
										}
,
										"valueof" : 										{
											"parameter_enum" : [ "val1", "val2" ],
											"parameter_longname" : "live.text[7]",
											"parameter_mmax" : 1,
											"parameter_modmode" : 0,
											"parameter_shortname" : "live.text",
											"parameter_type" : 2
										}

									}
,
									"text" : "bippity",
									"texton" : "boppity",
									"varname" : "live.text[9]"
								}

							}
, 							{
								"box" : 								{
									"fontsize" : 10.0,
									"id" : "obj-146",
									"linecount" : 3,
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 4,
									"outlettype" : [ "int", "", "", "" ],
									"patching_rect" : [ 389.0322265625, 337.7613525390625, 158.0, 42.0 ],
									"text" : "jk.push @pad 42 @oncolor r @offcolor g @texton boppity @textoff bippity @mode b"
								}

							}
, 							{
								"box" : 								{
									"activebgcolor" : [ 0.0, 1.0, 0.0, 1.0 ],
									"activebgoncolor" : [ 1.0, 0.0, 0.0, 1.0 ],
									"bordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
									"focusbordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
									"fontname" : "Arial",
									"fontsize" : 10.0,
									"id" : "obj-143",
									"maxclass" : "live.text",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "", "" ],
									"parameter_enable" : 1,
									"patching_rect" : [ 243.0322265625, 382.7613525390625, 76.266666666666666, 34.0 ],
									"saved_attribute_attributes" : 									{
										"activebgcolor" : 										{
											"expression" : ""
										}
,
										"activebgoncolor" : 										{
											"expression" : ""
										}
,
										"activetextcolor" : 										{
											"expression" : ""
										}
,
										"activetextoncolor" : 										{
											"expression" : ""
										}
,
										"bordercolor" : 										{
											"expression" : ""
										}
,
										"focusbordercolor" : 										{
											"expression" : ""
										}
,
										"valueof" : 										{
											"parameter_enum" : [ "val1", "val2" ],
											"parameter_longname" : "live.text[6]",
											"parameter_mmax" : 1,
											"parameter_modmode" : 0,
											"parameter_shortname" : "live.text",
											"parameter_type" : 2
										}

									}
,
									"text" : "bippity",
									"texton" : "boppity",
									"varname" : "live.text[8]"
								}

							}
, 							{
								"box" : 								{
									"fontsize" : 10.0,
									"id" : "obj-144",
									"linecount" : 3,
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 4,
									"outlettype" : [ "int", "", "", "" ],
									"patching_rect" : [ 243.0322265625, 337.7613525390625, 158.0, 42.0 ],
									"text" : "jk.push @pad 41 @oncolor r @offcolor g @texton boppity @textoff bippity @mode b"
								}

							}
, 							{
								"box" : 								{
									"activebgcolor" : [ 0.0, 1.0, 0.0, 1.0 ],
									"activebgoncolor" : [ 1.0, 0.0, 0.0, 1.0 ],
									"bordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
									"focusbordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
									"fontname" : "Arial",
									"fontsize" : 10.0,
									"id" : "obj-141",
									"maxclass" : "live.text",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "", "" ],
									"parameter_enable" : 1,
									"patching_rect" : [ 81.0322265625, 382.7613525390625, 76.266666666666666, 34.0 ],
									"saved_attribute_attributes" : 									{
										"activebgcolor" : 										{
											"expression" : ""
										}
,
										"activebgoncolor" : 										{
											"expression" : ""
										}
,
										"activetextcolor" : 										{
											"expression" : ""
										}
,
										"activetextoncolor" : 										{
											"expression" : ""
										}
,
										"bordercolor" : 										{
											"expression" : ""
										}
,
										"focusbordercolor" : 										{
											"expression" : ""
										}
,
										"valueof" : 										{
											"parameter_enum" : [ "val1", "val2" ],
											"parameter_longname" : "live.text[5]",
											"parameter_mmax" : 1,
											"parameter_modmode" : 0,
											"parameter_shortname" : "live.text",
											"parameter_type" : 2
										}

									}
,
									"text" : "bippity",
									"texton" : "boppity",
									"varname" : "live.text[7]"
								}

							}
, 							{
								"box" : 								{
									"fontsize" : 10.0,
									"id" : "obj-142",
									"linecount" : 3,
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 4,
									"outlettype" : [ "int", "", "", "" ],
									"patching_rect" : [ 81.0322265625, 337.600068330764771, 158.0, 42.0 ],
									"text" : "jk.push @pad 40 @oncolor r @offcolor g @texton boppity @textoff bippity @mode b"
								}

							}
, 							{
								"box" : 								{
									"activebgcolor" : [ 0.0, 1.0, 1.0, 1.0 ],
									"activebgoncolor" : [ 1.0, 0.0, 0.0, 1.0 ],
									"bordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
									"focusbordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
									"fontname" : "Arial",
									"fontsize" : 10.0,
									"id" : "obj-132",
									"maxclass" : "live.text",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "", "" ],
									"parameter_enable" : 1,
									"patching_rect" : [ 295.870952367782593, 142.438776612281799, 76.266666666666652, 34.0 ],
									"saved_attribute_attributes" : 									{
										"activebgcolor" : 										{
											"expression" : ""
										}
,
										"activebgoncolor" : 										{
											"expression" : ""
										}
,
										"activetextcolor" : 										{
											"expression" : ""
										}
,
										"activetextoncolor" : 										{
											"expression" : ""
										}
,
										"bordercolor" : 										{
											"expression" : ""
										}
,
										"focusbordercolor" : 										{
											"expression" : ""
										}
,
										"valueof" : 										{
											"parameter_enum" : [ "val1", "val2" ],
											"parameter_longname" : "live.text[1]",
											"parameter_mmax" : 1,
											"parameter_modmode" : 0,
											"parameter_shortname" : "live.text",
											"parameter_type" : 2
										}

									}
,
									"text" : "bippity",
									"texton" : "boppity",
									"varname" : "live.text[1]"
								}

							}
, 							{
								"box" : 								{
									"fontsize" : 10.0,
									"id" : "obj-133",
									"linecount" : 2,
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 4,
									"outlettype" : [ "int", "", "", "" ],
									"patching_rect" : [ 297.483855605125427, 106.954905390739441, 201.612904667854309, 31.0 ],
									"text" : "jk.push @pad 57 @oncolor r @offcolor c @texton boppity @textoff bippity @mode b"
								}

							}
, 							{
								"box" : 								{
									"activebgcolor" : [ 0.0, 1.0, 1.0, 1.0 ],
									"activebgoncolor" : [ 1.0, 0.0, 0.0, 1.0 ],
									"bordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
									"focusbordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
									"fontname" : "Arial",
									"fontsize" : 10.0,
									"id" : "obj-134",
									"maxclass" : "live.text",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "", "" ],
									"parameter_enable" : 1,
									"patching_rect" : [ 71.677402377128601, 142.438776612281799, 76.266666666666666, 34.0 ],
									"saved_attribute_attributes" : 									{
										"activebgcolor" : 										{
											"expression" : ""
										}
,
										"activebgoncolor" : 										{
											"expression" : ""
										}
,
										"activetextcolor" : 										{
											"expression" : ""
										}
,
										"activetextoncolor" : 										{
											"expression" : ""
										}
,
										"bordercolor" : 										{
											"expression" : ""
										}
,
										"focusbordercolor" : 										{
											"expression" : ""
										}
,
										"valueof" : 										{
											"parameter_enum" : [ "val1", "val2" ],
											"parameter_longname" : "live.text[2]",
											"parameter_mmax" : 1,
											"parameter_modmode" : 0,
											"parameter_shortname" : "live.text",
											"parameter_type" : 2
										}

									}
,
									"text" : "bippity",
									"texton" : "boppity",
									"varname" : "live.text[4]"
								}

							}
, 							{
								"box" : 								{
									"fontsize" : 10.0,
									"id" : "obj-135",
									"linecount" : 2,
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 4,
									"outlettype" : [ "int", "", "", "" ],
									"patching_rect" : [ 71.677402377128601, 106.954905390739441, 201.612904667854309, 31.0 ],
									"text" : "jk.push @pad 56 @oncolor r @offcolor c @texton boppity @textoff bippity @mode b"
								}

							}
, 							{
								"box" : 								{
									"activebgcolor" : [ 0.0, 1.0, 1.0, 1.0 ],
									"activebgoncolor" : [ 1.0, 0.0, 0.0, 1.0 ],
									"bordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
									"focusbordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
									"fontname" : "Arial",
									"fontsize" : 10.0,
									"id" : "obj-136",
									"maxclass" : "live.text",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "", "" ],
									"parameter_enable" : 1,
									"patching_rect" : [ 765.225794434547424, 144.051679849624634, 76.266666666666652, 34.0 ],
									"saved_attribute_attributes" : 									{
										"activebgcolor" : 										{
											"expression" : ""
										}
,
										"activebgoncolor" : 										{
											"expression" : ""
										}
,
										"activetextcolor" : 										{
											"expression" : ""
										}
,
										"activetextoncolor" : 										{
											"expression" : ""
										}
,
										"bordercolor" : 										{
											"expression" : ""
										}
,
										"focusbordercolor" : 										{
											"expression" : ""
										}
,
										"valueof" : 										{
											"parameter_enum" : [ "val1", "val2" ],
											"parameter_longname" : "live.text[3]",
											"parameter_mmax" : 1,
											"parameter_modmode" : 0,
											"parameter_shortname" : "live.text",
											"parameter_type" : 2
										}

									}
,
									"text" : "bippity",
									"texton" : "boppity",
									"varname" : "live.text[5]"
								}

							}
, 							{
								"box" : 								{
									"fontsize" : 10.0,
									"id" : "obj-137",
									"linecount" : 2,
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 4,
									"outlettype" : [ "int", "", "", "" ],
									"patching_rect" : [ 765.225794434547424, 106.954905390739441, 199.0, 31.0 ],
									"text" : "jk.push @pad 59 @oncolor r @offcolor c @texton boppity @textoff bippity @mode b"
								}

							}
, 							{
								"box" : 								{
									"activebgcolor" : [ 0.0, 1.0, 1.0, 1.0 ],
									"activebgoncolor" : [ 1.0, 0.0, 0.0, 1.0 ],
									"bordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
									"focusbordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
									"fontname" : "Arial",
									"fontsize" : 10.0,
									"id" : "obj-138",
									"maxclass" : "live.text",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "", "" ],
									"parameter_enable" : 1,
									"patching_rect" : [ 541.032244443893433, 144.051679849624634, 76.266666666666652, 34.0 ],
									"saved_attribute_attributes" : 									{
										"activebgcolor" : 										{
											"expression" : ""
										}
,
										"activebgoncolor" : 										{
											"expression" : ""
										}
,
										"activetextcolor" : 										{
											"expression" : ""
										}
,
										"activetextoncolor" : 										{
											"expression" : ""
										}
,
										"bordercolor" : 										{
											"expression" : ""
										}
,
										"focusbordercolor" : 										{
											"expression" : ""
										}
,
										"valueof" : 										{
											"parameter_enum" : [ "val1", "val2" ],
											"parameter_longname" : "live.text[4]",
											"parameter_mmax" : 1,
											"parameter_modmode" : 0,
											"parameter_shortname" : "live.text",
											"parameter_type" : 2
										}

									}
,
									"text" : "bippity",
									"texton" : "boppity",
									"varname" : "live.text[6]"
								}

							}
, 							{
								"box" : 								{
									"fontsize" : 10.0,
									"id" : "obj-139",
									"linecount" : 2,
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 4,
									"outlettype" : [ "int", "", "", "" ],
									"patching_rect" : [ 541.032244443893433, 106.954905390739441, 201.612904667854309, 31.0 ],
									"text" : "jk.push @pad 58 @oncolor r @offcolor c @texton boppity @textoff bippity @mode b"
								}

							}
, 							{
								"box" : 								{
									"activebgcolor" : [ 0.0, 1.0, 1.0, 1.0 ],
									"activebgoncolor" : [ 1.0, 0.0, 0.0, 1.0 ],
									"bordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
									"focusbordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
									"fontname" : "Arial",
									"fontsize" : 10.0,
									"id" : "obj-112",
									"maxclass" : "live.text",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "", "" ],
									"parameter_enable" : 1,
									"patching_rect" : [ 292.645145893096924, 221.471035242080688, 76.266666666666652, 34.0 ],
									"saved_attribute_attributes" : 									{
										"activebgcolor" : 										{
											"expression" : ""
										}
,
										"activebgoncolor" : 										{
											"expression" : ""
										}
,
										"activetextcolor" : 										{
											"expression" : ""
										}
,
										"activetextoncolor" : 										{
											"expression" : ""
										}
,
										"bordercolor" : 										{
											"expression" : ""
										}
,
										"focusbordercolor" : 										{
											"expression" : ""
										}
,
										"valueof" : 										{
											"parameter_enum" : [ "val1", "val2" ],
											"parameter_longname" : "live.text[70]",
											"parameter_mmax" : 1,
											"parameter_modmode" : 0,
											"parameter_shortname" : "live.text",
											"parameter_type" : 2
										}

									}
,
									"text" : "bippity",
									"texton" : "boppity",
									"varname" : "live.text[16]"
								}

							}
, 							{
								"box" : 								{
									"fontsize" : 10.0,
									"id" : "obj-113",
									"linecount" : 2,
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 4,
									"outlettype" : [ "int", "", "", "" ],
									"patching_rect" : [ 292.645145893096924, 184.374260783195496, 200.0, 31.0 ],
									"text" : "jk.push @pad 61 @oncolor r @offcolor c @texton boppity @textoff bippity @mode b"
								}

							}
, 							{
								"box" : 								{
									"activebgcolor" : [ 0.0, 1.0, 1.0, 1.0 ],
									"activebgoncolor" : [ 1.0, 0.0, 0.0, 1.0 ],
									"bordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
									"focusbordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
									"fontname" : "Arial",
									"fontsize" : 10.0,
									"id" : "obj-114",
									"maxclass" : "live.text",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "", "" ],
									"parameter_enable" : 1,
									"patching_rect" : [ 68.451595902442932, 221.471035242080688, 76.266666666666666, 34.0 ],
									"saved_attribute_attributes" : 									{
										"activebgcolor" : 										{
											"expression" : ""
										}
,
										"activebgoncolor" : 										{
											"expression" : ""
										}
,
										"activetextcolor" : 										{
											"expression" : ""
										}
,
										"activetextoncolor" : 										{
											"expression" : ""
										}
,
										"bordercolor" : 										{
											"expression" : ""
										}
,
										"focusbordercolor" : 										{
											"expression" : ""
										}
,
										"valueof" : 										{
											"parameter_enum" : [ "val1", "val2" ],
											"parameter_longname" : "live.text[71]",
											"parameter_mmax" : 1,
											"parameter_modmode" : 0,
											"parameter_shortname" : "live.text",
											"parameter_type" : 2
										}

									}
,
									"text" : "bippity",
									"texton" : "boppity",
									"varname" : "live.text[17]"
								}

							}
, 							{
								"box" : 								{
									"fontsize" : 10.0,
									"id" : "obj-115",
									"linecount" : 2,
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 4,
									"outlettype" : [ "int", "", "", "" ],
									"patching_rect" : [ 68.451595902442932, 184.374260783195496, 200.0, 31.0 ],
									"text" : "jk.push @pad 60 @oncolor r @offcolor c @texton boppity @textoff bippity @mode b"
								}

							}
, 							{
								"box" : 								{
									"activebgcolor" : [ 0.0, 1.0, 1.0, 1.0 ],
									"activebgoncolor" : [ 1.0, 0.0, 0.0, 1.0 ],
									"bordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
									"focusbordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
									"fontname" : "Arial",
									"fontsize" : 10.0,
									"id" : "obj-116",
									"maxclass" : "live.text",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "", "" ],
									"parameter_enable" : 1,
									"patching_rect" : [ 739.419342637062073, 221.471035242080688, 76.266666666666652, 34.0 ],
									"saved_attribute_attributes" : 									{
										"activebgcolor" : 										{
											"expression" : ""
										}
,
										"activebgoncolor" : 										{
											"expression" : ""
										}
,
										"activetextcolor" : 										{
											"expression" : ""
										}
,
										"activetextoncolor" : 										{
											"expression" : ""
										}
,
										"bordercolor" : 										{
											"expression" : ""
										}
,
										"focusbordercolor" : 										{
											"expression" : ""
										}
,
										"valueof" : 										{
											"parameter_enum" : [ "val1", "val2" ],
											"parameter_longname" : "live.text[78]",
											"parameter_mmax" : 1,
											"parameter_modmode" : 0,
											"parameter_shortname" : "live.text",
											"parameter_type" : 2
										}

									}
,
									"text" : "bippity",
									"texton" : "boppity",
									"varname" : "live.text[2]"
								}

							}
, 							{
								"box" : 								{
									"fontsize" : 10.0,
									"id" : "obj-117",
									"linecount" : 2,
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 4,
									"outlettype" : [ "int", "", "", "" ],
									"patching_rect" : [ 739.419342637062073, 184.374260783195496, 199.0, 31.0 ],
									"text" : "jk.push @pad 63 @oncolor r @offcolor c @texton boppity @textoff bippity @mode b"
								}

							}
, 							{
								"box" : 								{
									"activebgcolor" : [ 0.0, 1.0, 1.0, 1.0 ],
									"activebgoncolor" : [ 1.0, 0.0, 0.0, 1.0 ],
									"bordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
									"focusbordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
									"fontname" : "Arial",
									"fontsize" : 10.0,
									"id" : "obj-118",
									"maxclass" : "live.text",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "", "" ],
									"parameter_enable" : 1,
									"patching_rect" : [ 513.612889409065247, 221.471035242080688, 76.266666666666652, 34.0 ],
									"saved_attribute_attributes" : 									{
										"activebgcolor" : 										{
											"expression" : ""
										}
,
										"activebgoncolor" : 										{
											"expression" : ""
										}
,
										"activetextcolor" : 										{
											"expression" : ""
										}
,
										"activetextoncolor" : 										{
											"expression" : ""
										}
,
										"bordercolor" : 										{
											"expression" : ""
										}
,
										"focusbordercolor" : 										{
											"expression" : ""
										}
,
										"valueof" : 										{
											"parameter_enum" : [ "val1", "val2" ],
											"parameter_longname" : "live.text[79]",
											"parameter_mmax" : 1,
											"parameter_modmode" : 0,
											"parameter_shortname" : "live.text",
											"parameter_type" : 2
										}

									}
,
									"text" : "bippity",
									"texton" : "boppity",
									"varname" : "live.text[3]"
								}

							}
, 							{
								"box" : 								{
									"fontsize" : 10.0,
									"id" : "obj-119",
									"linecount" : 2,
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 4,
									"outlettype" : [ "int", "", "", "" ],
									"patching_rect" : [ 513.612889409065247, 184.374260783195496, 202.0, 31.0 ],
									"text" : "jk.push @pad 62 @oncolor r @offcolor c @texton boppity @textoff bippity @mode b"
								}

							}
 ],
						"lines" : [ 							{
								"patchline" : 								{
									"destination" : [ "obj-113", 0 ],
									"midpoints" : [ 302.145145893096924, 256.156057834625244, 288.999986529350281, 256.156057834625244, 288.999986529350281, 181.156057834625244, 302.145145893096924, 181.156057834625244 ],
									"order" : 1,
									"source" : [ "obj-112", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-28", 0 ],
									"order" : 0,
									"source" : [ "obj-112", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-112", 0 ],
									"source" : [ "obj-113", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-115", 0 ],
									"midpoints" : [ 77.951595902442932, 256.156057834625244, 51.999986529350281, 256.156057834625244, 51.999986529350281, 181.156057834625244, 77.951595902442932, 181.156057834625244 ],
									"order" : 1,
									"source" : [ "obj-114", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-28", 0 ],
									"order" : 0,
									"source" : [ "obj-114", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-114", 0 ],
									"source" : [ "obj-115", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-117", 0 ],
									"midpoints" : [ 748.919342637062073, 256.156057834625244, 735.999986529350281, 256.156057834625244, 735.999986529350281, 181.156057834625244, 748.919342637062073, 181.156057834625244 ],
									"order" : 2,
									"source" : [ "obj-116", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-24", 1 ],
									"order" : 0,
									"source" : [ "obj-116", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-28", 0 ],
									"order" : 1,
									"source" : [ "obj-116", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-116", 0 ],
									"source" : [ "obj-117", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-119", 0 ],
									"midpoints" : [ 523.112889409065247, 256.156057834625244, 510.999986529350281, 256.156057834625244, 510.999986529350281, 181.156057834625244, 523.112889409065247, 181.156057834625244 ],
									"order" : 1,
									"source" : [ "obj-118", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-28", 0 ],
									"order" : 0,
									"source" : [ "obj-118", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-118", 0 ],
									"source" : [ "obj-119", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-133", 0 ],
									"midpoints" : [ 305.370952367782593, 177.00000262260437, 293.032266616821289, 177.00000262260437, 293.032266616821289, 102.00000262260437, 306.983855605125427, 102.00000262260437 ],
									"order" : 1,
									"source" : [ "obj-132", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-28", 0 ],
									"order" : 0,
									"source" : [ "obj-132", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-132", 0 ],
									"source" : [ "obj-133", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-135", 0 ],
									"midpoints" : [ 81.177402377128601, 177.00000262260437, 56.032266616821289, 177.00000262260437, 56.032266616821289, 102.00000262260437, 81.177402377128601, 102.00000262260437 ],
									"order" : 1,
									"source" : [ "obj-134", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-28", 0 ],
									"order" : 0,
									"source" : [ "obj-134", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-134", 0 ],
									"source" : [ "obj-135", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-137", 0 ],
									"midpoints" : [ 774.725794434547424, 177.00000262260437, 763.225792109966278, 177.00000262260437, 763.225792109966278, 102.00000262260437, 774.725794434547424, 102.00000262260437 ],
									"order" : 1,
									"source" : [ "obj-136", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-28", 0 ],
									"order" : 0,
									"source" : [ "obj-136", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-136", 0 ],
									"source" : [ "obj-137", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-139", 0 ],
									"midpoints" : [ 550.532244443893433, 177.00000262260437, 538.225792109966278, 177.00000262260437, 538.225792109966278, 102.00000262260437, 550.532244443893433, 102.00000262260437 ],
									"order" : 1,
									"source" : [ "obj-138", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-28", 0 ],
									"order" : 0,
									"source" : [ "obj-138", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-138", 0 ],
									"source" : [ "obj-139", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-142", 0 ],
									"midpoints" : [ 90.5322265625, 407.6451655626297, 65.709686040878296, 407.6451655626297, 65.709686040878296, 332.6451655626297, 90.5322265625, 332.6451655626297 ],
									"order" : 1,
									"source" : [ "obj-141", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-29", 0 ],
									"order" : 0,
									"source" : [ "obj-141", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-141", 0 ],
									"source" : [ "obj-142", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-144", 0 ],
									"midpoints" : [ 252.5322265625, 407.806449770927429, 227.709686040878296, 407.806449770927429, 227.709686040878296, 332.806449770927429, 252.5322265625, 332.806449770927429 ],
									"order" : 1,
									"source" : [ "obj-143", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-29", 0 ],
									"order" : 0,
									"source" : [ "obj-143", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-143", 0 ],
									"source" : [ "obj-144", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-146", 0 ],
									"midpoints" : [ 398.5322265625, 407.806449770927429, 373.709686040878296, 407.806449770927429, 373.709686040878296, 332.806449770927429, 398.5322265625, 332.806449770927429 ],
									"order" : 1,
									"source" : [ "obj-145", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-29", 0 ],
									"order" : 0,
									"source" : [ "obj-145", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-145", 0 ],
									"source" : [ "obj-146", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-148", 0 ],
									"midpoints" : [ 585.112889409065247, 399.806449770927429, 560.290348887443542, 399.806449770927429, 560.290348887443542, 324.806449770927429, 585.5322265625, 324.806449770927429 ],
									"order" : 1,
									"source" : [ "obj-147", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-29", 0 ],
									"order" : 0,
									"source" : [ "obj-147", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-147", 0 ],
									"source" : [ "obj-148", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-150", 0 ],
									"midpoints" : [ 90.5322265625, 504.806449770927429, 65.709686040878296, 504.806449770927429, 65.709686040878296, 429.806449770927429, 90.5322265625, 429.806449770927429 ],
									"order" : 1,
									"source" : [ "obj-149", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-29", 0 ],
									"order" : 0,
									"source" : [ "obj-149", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-149", 0 ],
									"source" : [ "obj-150", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-152", 0 ],
									"midpoints" : [ 259.5322265625, 504.806449770927429, 234.709686040878296, 504.806449770927429, 234.709686040878296, 429.806449770927429, 259.5322265625, 429.806449770927429 ],
									"order" : 1,
									"source" : [ "obj-151", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-29", 0 ],
									"order" : 0,
									"source" : [ "obj-151", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-151", 0 ],
									"source" : [ "obj-152", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-154", 0 ],
									"midpoints" : [ 422.5322265625, 504.806449770927429, 397.709686040878296, 504.806449770927429, 397.709686040878296, 429.806449770927429, 422.5322265625, 429.806449770927429 ],
									"order" : 1,
									"source" : [ "obj-153", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-29", 0 ],
									"order" : 0,
									"source" : [ "obj-153", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-153", 0 ],
									"source" : [ "obj-154", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-156", 0 ],
									"midpoints" : [ 585.112889409065247, 500.806449770927429, 560.290348887443542, 500.806449770927429, 560.290348887443542, 425.806449770927429, 585.5322265625, 425.806449770927429 ],
									"order" : 1,
									"source" : [ "obj-155", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-29", 0 ],
									"order" : 0,
									"source" : [ "obj-155", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-155", 0 ],
									"source" : [ "obj-156", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-158", 0 ],
									"midpoints" : [ 82.5322265625, 600.806449770927429, 57.644422888755798, 600.806449770927429, 57.644422888755798, 525.806449770927429, 82.5322265625, 525.806449770927429 ],
									"order" : 1,
									"source" : [ "obj-157", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-30", 0 ],
									"order" : 0,
									"source" : [ "obj-157", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-157", 0 ],
									"source" : [ "obj-158", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-160", 0 ],
									"midpoints" : [ 292.5322265625, 600.806449770927429, 266.096051931381226, 600.806449770927429, 266.096051931381226, 525.806449770927429, 290.5322265625, 525.806449770927429 ],
									"order" : 1,
									"source" : [ "obj-159", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-30", 0 ],
									"order" : 0,
									"source" : [ "obj-159", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-7", 0 ],
									"source" : [ "obj-16", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-159", 0 ],
									"source" : [ "obj-160", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-162", 0 ],
									"midpoints" : [ 484.5322265625, 600.806449770927429, 457.491752401987469, 600.806449770927429, 457.491752401987469, 525.806449770927429, 482.5322265625, 525.806449770927429 ],
									"order" : 1,
									"source" : [ "obj-161", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-30", 0 ],
									"order" : 0,
									"source" : [ "obj-161", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-161", 0 ],
									"source" : [ "obj-162", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-164", 0 ],
									"midpoints" : [ 688.5322265625, 604.412412405014038, 664.664872296651083, 604.412412405014038, 664.664872296651083, 529.412412405014038, 688.5322265625, 529.412412405014038 ],
									"order" : 1,
									"source" : [ "obj-163", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-30", 0 ],
									"order" : 0,
									"source" : [ "obj-163", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-163", 0 ],
									"source" : [ "obj-164", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-166", 0 ],
									"midpoints" : [ 688.5322265625, 718.624425888061523, 664.025610693295675, 718.624425888061523, 664.025610693295675, 643.624425888061523, 688.5322265625, 643.624425888061523 ],
									"order" : 1,
									"source" : [ "obj-165", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-30", 0 ],
									"order" : 0,
									"source" : [ "obj-165", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-165", 0 ],
									"source" : [ "obj-166", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-168", 0 ],
									"midpoints" : [ 482.5322265625, 715.018463253974915, 456.852490798632061, 715.018463253974915, 456.852490798632061, 640.018463253974915, 480.5322265625, 640.018463253974915 ],
									"order" : 1,
									"source" : [ "obj-167", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-30", 0 ],
									"order" : 0,
									"source" : [ "obj-167", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-167", 0 ],
									"source" : [ "obj-168", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-170", 0 ],
									"midpoints" : [ 290.5322265625, 715.018463253974915, 265.456790328025818, 715.018463253974915, 265.456790328025818, 640.018463253974915, 290.5322265625, 640.018463253974915 ],
									"order" : 1,
									"source" : [ "obj-169", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-30", 0 ],
									"order" : 0,
									"source" : [ "obj-169", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-169", 0 ],
									"source" : [ "obj-170", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-172", 0 ],
									"midpoints" : [ 82.5322265625, 715.018463253974915, 57.005161285400391, 715.018463253974915, 57.005161285400391, 640.018463253974915, 80.5322265625, 640.018463253974915 ],
									"order" : 1,
									"source" : [ "obj-171", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-30", 0 ],
									"order" : 0,
									"source" : [ "obj-171", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-171", 0 ],
									"source" : [ "obj-172", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-7", 0 ],
									"source" : [ "obj-18", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-190", 0 ],
									"midpoints" : [ 79.774650871753693, 847.092151522636414, 54.886847198009491, 847.092151522636414, 54.886847198009491, 772.092151522636414, 79.880013942718506, 772.092151522636414 ],
									"order" : 1,
									"source" : [ "obj-189", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-31", 0 ],
									"order" : 0,
									"source" : [ "obj-189", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-189", 0 ],
									"source" : [ "obj-190", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-192", 0 ],
									"midpoints" : [ 273.35827112197876, 847.092151522636414, 248.365104377269745, 847.092151522636414, 248.365104377269745, 772.092151522636414, 273.35827112197876, 772.092151522636414 ],
									"order" : 1,
									"source" : [ "obj-191", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-31", 0 ],
									"order" : 0,
									"source" : [ "obj-191", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-191", 0 ],
									"source" : [ "obj-192", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-194", 0 ],
									"midpoints" : [ 665.403403655687725, 843.023816466331482, 640.410236910978711, 843.023816466331482, 640.410236910978711, 768.023816466331482, 665.403403655687725, 768.023816466331482 ],
									"order" : 1,
									"source" : [ "obj-193", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-31", 0 ],
									"order" : 0,
									"source" : [ "obj-193", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-193", 0 ],
									"source" : [ "obj-194", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-196", 0 ],
									"midpoints" : [ 471.925146476427471, 843.023816466331482, 446.931979731718457, 843.023816466331482, 446.931979731718457, 768.023816466331482, 471.925146476427471, 768.023816466331482 ],
									"order" : 1,
									"source" : [ "obj-195", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-31", 0 ],
									"order" : 0,
									"source" : [ "obj-195", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-195", 0 ],
									"source" : [ "obj-196", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-198", 0 ],
									"midpoints" : [ 664.662622213363647, 938.593198776245117, 639.685593918959057, 938.593198776245117, 639.685593918959057, 863.593198776245117, 664.662622213363647, 863.593198776245117 ],
									"order" : 1,
									"source" : [ "obj-197", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-31", 0 ],
									"order" : 0,
									"source" : [ "obj-197", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-197", 0 ],
									"source" : [ "obj-198", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-200", 0 ],
									"midpoints" : [ 471.908997416496277, 938.593198776245117, 446.207336739698803, 938.593198776245117, 446.207336739698803, 863.593198776245117, 471.908997416496277, 863.593198776245117 ],
									"order" : 1,
									"source" : [ "obj-199", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-31", 0 ],
									"order" : 0,
									"source" : [ "obj-199", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-7", 0 ],
									"source" : [ "obj-20", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-199", 0 ],
									"source" : [ "obj-200", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-202", 0 ],
									"midpoints" : [ 273.35827112197876, 942.661533832550049, 247.640461385250092, 942.661533832550049, 247.640461385250092, 867.661533832550049, 273.35827112197876, 867.661533832550049 ],
									"order" : 1,
									"source" : [ "obj-201", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-31", 0 ],
									"order" : 0,
									"source" : [ "obj-201", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-201", 0 ],
									"source" : [ "obj-202", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-204", 0 ],
									"midpoints" : [ 79.155370950698853, 942.661533832550049, 54.162204205989838, 942.661533832550049, 54.162204205989838, 867.661533832550049, 79.155370950698853, 867.661533832550049 ],
									"order" : 1,
									"source" : [ "obj-203", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-31", 0 ],
									"order" : 0,
									"source" : [ "obj-203", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-203", 0 ],
									"source" : [ "obj-204", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-7", 0 ],
									"source" : [ "obj-22", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-16", 0 ],
									"source" : [ "obj-28", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-18", 0 ],
									"source" : [ "obj-29", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-20", 0 ],
									"source" : [ "obj-30", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-22", 0 ],
									"source" : [ "obj-31", 0 ]
								}

							}
 ]
					}
,
					"patching_rect" : [ 2030.357123494148254, 683.749934792518616, 64.262876505851636, 22.0 ],
					"text" : "p"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-39",
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 2188.0, 859.130406618118286, 29.5, 22.0 ],
					"text" : "1"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-252",
					"linecount" : 5,
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 4,
					"outlettype" : [ "int", "", "", "" ],
					"patching_rect" : [ 2234.146394729614258, 1084.146367311477661, 142.68293023109436, 76.0 ],
					"text" : "jk.push @button scene7 @oncolor w @offcolor b @textoff -18dB @texton -18dB @mode r @radio volumes",
					"varname" : "jkpush16[1]"
				}

			}
, 			{
				"box" : 				{
					"activebgcolor" : [ 0.0, 0.0, 1.0, 1.0 ],
					"activebgoncolor" : [ 1.0, 1.0, 1.0, 1.0 ],
					"activetextcolor" : [ 1.0, 1.0, 1.0, 1.0 ],
					"bordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
					"focusbordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
					"fontname" : "Arial Narrow",
					"fontsize" : 11.0,
					"id" : "obj-33",
					"maxclass" : "live.text",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 2187.804930210113525, 1084.146367311477661, 44.0, 15.0 ],
					"saved_attribute_attributes" : 					{
						"activebgcolor" : 						{
							"expression" : ""
						}
,
						"activebgoncolor" : 						{
							"expression" : ""
						}
,
						"activetextcolor" : 						{
							"expression" : ""
						}
,
						"activetextoncolor" : 						{
							"expression" : ""
						}
,
						"bordercolor" : 						{
							"expression" : ""
						}
,
						"focusbordercolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_enum" : [ "val1", "val2" ],
							"parameter_longname" : "pushtext16[1]",
							"parameter_mmax" : 1,
							"parameter_modmode" : 0,
							"parameter_shortname" : "pushtext16",
							"parameter_type" : 2
						}

					}
,
					"text" : "-18dB",
					"texton" : "-18dB",
					"varname" : "pushtext16[1]"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-258",
					"linecount" : 5,
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 4,
					"outlettype" : [ "int", "", "", "" ],
					"patching_rect" : [ 2234.146394729614258, 1003.658560514450073, 142.68293023109436, 76.0 ],
					"text" : "jk.push @button scene5 @oncolor w @offcolor b @textoff -12dB @texton -12dB @mode r @radio volumes",
					"varname" : "jkpush18[1]"
				}

			}
, 			{
				"box" : 				{
					"activebgcolor" : [ 0.0, 0.0, 1.0, 1.0 ],
					"activebgoncolor" : [ 1.0, 1.0, 1.0, 1.0 ],
					"activetextcolor" : [ 1.0, 1.0, 1.0, 1.0 ],
					"bordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
					"focusbordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
					"fontname" : "Arial Narrow",
					"fontsize" : 11.0,
					"id" : "obj-35",
					"maxclass" : "live.text",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 2187.804930210113525, 1003.658560514450073, 44.0, 15.0 ],
					"saved_attribute_attributes" : 					{
						"activebgcolor" : 						{
							"expression" : ""
						}
,
						"activebgoncolor" : 						{
							"expression" : ""
						}
,
						"activetextcolor" : 						{
							"expression" : ""
						}
,
						"activetextoncolor" : 						{
							"expression" : ""
						}
,
						"bordercolor" : 						{
							"expression" : ""
						}
,
						"focusbordercolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_enum" : [ "val1", "val2" ],
							"parameter_longname" : "pushtext18[1]",
							"parameter_mmax" : 1,
							"parameter_modmode" : 0,
							"parameter_shortname" : "pushtext18",
							"parameter_type" : 2
						}

					}
,
					"text" : "-12dB",
					"texton" : "-12dB",
					"varname" : "pushtext18[1]"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-264",
					"linecount" : 5,
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 4,
					"outlettype" : [ "int", "", "", "" ],
					"patching_rect" : [ 2234.146394729614258, 920.869535028934479, 142.68293023109436, 76.0 ],
					"text" : "jk.push @button scene3 @oncolor w @offcolor b @textoff -6dB @texton -6dB @mode r @radio volumes",
					"varname" : "jkpush20[1]"
				}

			}
, 			{
				"box" : 				{
					"activebgcolor" : [ 0.0, 0.0, 1.0, 1.0 ],
					"activebgoncolor" : [ 1.0, 1.0, 1.0, 1.0 ],
					"activetextcolor" : [ 1.0, 1.0, 1.0, 1.0 ],
					"bordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
					"focusbordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
					"fontname" : "Arial Narrow",
					"fontsize" : 11.0,
					"id" : "obj-20",
					"maxclass" : "live.text",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 2187.82601523399353, 920.869535028934479, 44.0, 15.0 ],
					"saved_attribute_attributes" : 					{
						"activebgcolor" : 						{
							"expression" : ""
						}
,
						"activebgoncolor" : 						{
							"expression" : ""
						}
,
						"activetextcolor" : 						{
							"expression" : ""
						}
,
						"activetextoncolor" : 						{
							"expression" : ""
						}
,
						"bordercolor" : 						{
							"expression" : ""
						}
,
						"focusbordercolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_enum" : [ "val1", "val2" ],
							"parameter_longname" : "pushtext20[1]",
							"parameter_mmax" : 1,
							"parameter_modmode" : 0,
							"parameter_shortname" : "pushtext20",
							"parameter_type" : 2
						}

					}
,
					"text" : "-6dB",
					"texton" : "-6dB",
					"varname" : "pushtext20[1]"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-270",
					"linecount" : 5,
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 4,
					"outlettype" : [ "int", "", "", "" ],
					"patching_rect" : [ 2234.146394729614258, 840.0, 142.68293023109436, 76.0 ],
					"text" : "jk.push @button scene1 @oncolor w @offcolor b @textoff 0dB @texton 0dB @mode r @radio volumes",
					"varname" : "jkpush22[1]"
				}

			}
, 			{
				"box" : 				{
					"activebgcolor" : [ 0.0, 0.0, 1.0, 1.0 ],
					"activebgoncolor" : [ 1.0, 1.0, 1.0, 1.0 ],
					"activetextcolor" : [ 1.0, 1.0, 1.0, 1.0 ],
					"bordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
					"focusbordercolor" : [ 0.0, 0.0, 0.0, 0.0 ],
					"fontname" : "Arial Narrow",
					"fontsize" : 11.0,
					"id" : "obj-37",
					"maxclass" : "live.text",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 2188.0, 840.0, 44.0, 15.0 ],
					"saved_attribute_attributes" : 					{
						"activebgcolor" : 						{
							"expression" : ""
						}
,
						"activebgoncolor" : 						{
							"expression" : ""
						}
,
						"activetextcolor" : 						{
							"expression" : ""
						}
,
						"activetextoncolor" : 						{
							"expression" : ""
						}
,
						"bordercolor" : 						{
							"expression" : ""
						}
,
						"focusbordercolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_enum" : [ "val1", "val2" ],
							"parameter_longname" : "pushtext22[1]",
							"parameter_mmax" : 1,
							"parameter_modmode" : 0,
							"parameter_shortname" : "pushtext22",
							"parameter_type" : 2
						}

					}
,
					"text" : "0dB",
					"texton" : "0dB",
					"varname" : "pushtext22[1]"
				}

			}
, 			{
				"box" : 				{
					"activedialcolor" : [ 1.0, 0.0, 0.0, 1.0 ],
					"activefgdialcolor" : [ 0.0, 0.0, 1.0, 1.0 ],
					"activeneedlecolor" : [ 1.0, 1.0, 0.0, 1.0 ],
					"fgdialcolor" : [ 0.243137254901961, 0.698039215686274, 0.415686274509804, 1.0 ],
					"focusbordercolor" : [ 0.65098, 0.666667, 0.662745, 1.0 ],
					"fontname" : "Arial",
					"fontsize" : 10.0,
					"id" : "obj-213",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1917.142902851104736, 1314.285745620727539, 27.0, 48.0 ],
					"saved_attribute_attributes" : 					{
						"activedialcolor" : 						{
							"expression" : ""
						}
,
						"activefgdialcolor" : 						{
							"expression" : ""
						}
,
						"activeneedlecolor" : 						{
							"expression" : ""
						}
,
						"fgdialcolor" : 						{
							"expression" : ""
						}
,
						"focusbordercolor" : 						{
							"expression" : ""
						}
,
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "2[2]",
							"parameter_mmax" : 1000.0,
							"parameter_modmode" : 0,
							"parameter_shortname" : "2",
							"parameter_type" : 0,
							"parameter_unitstyle" : 0
						}

					}
,
					"varname" : "live.dial[29]"
				}

			}
, 			{
				"box" : 				{
					"activedialcolor" : [ 1.0, 0.0, 0.0, 1.0 ],
					"activefgdialcolor" : [ 0.0, 0.0, 1.0, 1.0 ],
					"activeneedlecolor" : [ 1.0, 1.0, 0.0, 1.0 ],
					"fgdialcolor" : [ 0.243137254901961, 0.698039215686274, 0.415686274509804, 1.0 ],
					"focusbordercolor" : [ 0.65098, 0.666667, 0.662745, 1.0 ],
					"fontname" : "Arial",
					"fontsize" : 10.0,
					"id" : "obj-212",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1838.571472406387329, 1314.285745620727539, 27.0, 48.0 ],
					"saved_attribute_attributes" : 					{
						"activedialcolor" : 						{
							"expression" : ""
						}
,
						"activefgdialcolor" : 						{
							"expression" : ""
						}
,
						"activeneedlecolor" : 						{
							"expression" : ""
						}
,
						"fgdialcolor" : 						{
							"expression" : ""
						}
,
						"focusbordercolor" : 						{
							"expression" : ""
						}
,
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "2[1]",
							"parameter_mmax" : 1000.0,
							"parameter_modmode" : 0,
							"parameter_shortname" : "2",
							"parameter_type" : 0,
							"parameter_unitstyle" : 0
						}

					}
,
					"varname" : "live.dial[30]"
				}

			}
, 			{
				"box" : 				{
					"activedialcolor" : [ 1.0, 0.0, 0.0, 1.0 ],
					"activefgdialcolor" : [ 0.0, 0.0, 1.0, 1.0 ],
					"activeneedlecolor" : [ 1.0, 1.0, 0.0, 1.0 ],
					"fgdialcolor" : [ 0.243137254901961, 0.698039215686274, 0.415686274509804, 1.0 ],
					"focusbordercolor" : [ 0.65098, 0.666667, 0.662745, 1.0 ],
					"fontname" : "Arial",
					"fontsize" : 10.0,
					"id" : "obj-181",
					"maxclass" : "live.dial",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "float" ],
					"parameter_enable" : 1,
					"patching_rect" : [ 1761.428613424301147, 1314.285745620727539, 27.0, 48.0 ],
					"saved_attribute_attributes" : 					{
						"activedialcolor" : 						{
							"expression" : ""
						}
,
						"activefgdialcolor" : 						{
							"expression" : ""
						}
,
						"activeneedlecolor" : 						{
							"expression" : ""
						}
,
						"fgdialcolor" : 						{
							"expression" : ""
						}
,
						"focusbordercolor" : 						{
							"expression" : ""
						}
,
						"textcolor" : 						{
							"expression" : ""
						}
,
						"valueof" : 						{
							"parameter_longname" : "2",
							"parameter_mmax" : 1000.0,
							"parameter_modmode" : 0,
							"parameter_shortname" : "2",
							"parameter_type" : 0,
							"parameter_unitstyle" : 0
						}

					}

				}

			}
, 			{
				"box" : 				{
					"fontsize" : 10.0,
					"id" : "obj-210",
					"linecount" : 4,
					"maxclass" : "newobj",
					"numinlets" : 6,
					"numoutlets" : 4,
					"outlettype" : [ "float", "float", "float", "float" ],
					"patching_rect" : [ 1761.016991138458252, 1251.928559184074402, 262.711870670318604, 53.0 ],
					"text" : "jk.pushrotaryAlt 3 @dial1 r @dial2 b @init 500 @needle y @dialtextcolor k @bton y @btoff b @oncolor y @offcolor b @texton \"3\" @textoff \"Rotary 3\" @lo 0 @hi 1000 @inc 1"
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 10.0,
					"id" : "obj-120",
					"items" : [ "IAC Driver Bus 1", ",", "Euphonix MIDI Euphonix Port 1", ",", "Euphonix MIDI Euphonix Port 2", ",", "Euphonix MIDI Euphonix Port 3", ",", "Euphonix MIDI Euphonix Port 4", ",", "to Max 1", ",", "to Max 2", ",", "UltraLite-mk4 MIDI In", ",", "Ableton Push 2 Live Port", ",", "Ableton Push 2 User Port" ],
					"maxclass" : "umenu",
					"numinlets" : 1,
					"numoutlets" : 3,
					"outlettype" : [ "int", "", "" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 2182.0, 473.529404640197754, 100.0, 20.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 1619.44452166557312, 1865.277866721153259, 100.0, 20.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-121",
					"maxclass" : "toggle",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "int" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 2274.0, 377.529404640197754, 24.0, 24.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 1518.055627942085266, 1836.111198663711548, 24.0, 24.0 ]
				}

			}
, 			{
				"box" : 				{
					"bubble" : 1,
					"fontsize" : 10.0,
					"id" : "obj-122",
					"linecount" : 2,
					"maxclass" : "comment",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 2300.0, 373.529404640197754, 270.0, 33.0 ],
					"text" : "\"On\" blacks out the Push, \"Off\" resets.\n(Lightshow must be off for this to work.)"
				}

			}
, 			{
				"box" : 				{
					"disabled" : [ 0, 0, 0 ],
					"id" : "obj-123",
					"itemtype" : 0,
					"maxclass" : "radiogroup",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 2240.0, 349.529404640197754, 18.0, 50.0 ],
					"size" : 3,
					"value" : 0
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 10.0,
					"id" : "obj-124",
					"maxclass" : "number",
					"minimum" : 1,
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "bang" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 2186.0, 349.529404640197754, 50.0, 20.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-125",
					"maxclass" : "toggle",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "int" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 2156.0, 349.529404640197754, 24.0, 24.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-126",
					"maxclass" : "button",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "bang" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 2080.0, 391.529404640197754, 24.0, 24.0 ]
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 10.0,
					"id" : "obj-127",
					"items" : [ "AU DLS Synth 1", ",", "IAC Driver Bus 1", ",", "Euphonix MIDI Euphonix Port 1", ",", "Euphonix MIDI Euphonix Port 2", ",", "Euphonix MIDI Euphonix Port 3", ",", "Euphonix MIDI Euphonix Port 4", ",", "from Max 1", ",", "from Max 2", ",", "UltraLite-mk4 MIDI Out", ",", "Ableton Push 2 Live Port", ",", "Ableton Push 2 User Port" ],
					"maxclass" : "umenu",
					"numinlets" : 1,
					"numoutlets" : 3,
					"outlettype" : [ "int", "", "" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 2080.0, 473.529404640197754, 100.0, 20.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 1518.055627942085266, 1865.277866721153259, 100.0, 20.0 ]
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 10.0,
					"id" : "obj-22",
					"maxclass" : "newobj",
					"numinlets" : 7,
					"numoutlets" : 2,
					"outlettype" : [ "", "" ],
					"patching_rect" : [ 2080.0, 439.529404640197754, 71.5, 20.0 ],
					"text" : "jk.pushcore"
				}

			}
, 			{
				"box" : 				{
					"bubble" : 1,
					"bubbleside" : 2,
					"fontsize" : 11.0,
					"id" : "obj-129",
					"maxclass" : "comment",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 2050.0, 349.529404640197754, 82.0, 38.0 ],
					"text" : "Reset/Initialize"
				}

			}
, 			{
				"box" : 				{
					"args" : [ "tenor" ],
					"bgmode" : 0,
					"border" : 0,
					"clickthrough" : 0,
					"enablehscroll" : 0,
					"enablevscroll" : 0,
					"id" : "obj-16",
					"lockeddragscroll" : 0,
					"lockedsize" : 0,
					"maxclass" : "bpatcher",
					"name" : "gridmodule.maxpat",
					"numinlets" : 0,
					"numoutlets" : 2,
					"offset" : [ 0.0, 0.0 ],
					"outlettype" : [ "", "" ],
					"patching_rect" : [ 712.857159852981567, 352.85715126991272, 329.333343148231506, 222.290322661399841 ],
					"presentation" : 1,
					"presentation_rect" : [ 717.073187828063965, 286.585372686386108, 329.333343148231506, 222.290322661399841 ],
					"viewvisibility" : 1
				}

			}
, 			{
				"box" : 				{
					"bgmode" : 0,
					"border" : 0,
					"clickthrough" : 0,
					"enablehscroll" : 0,
					"enablevscroll" : 0,
					"id" : "obj-17",
					"lockeddragscroll" : 0,
					"lockedsize" : 0,
					"maxclass" : "bpatcher",
					"name" : "MCLyra.maxpat",
					"numinlets" : 2,
					"numoutlets" : 1,
					"offset" : [ 0.0, 0.0 ],
					"outlettype" : [ "multichannelsignal" ],
					"patching_rect" : [ 410.000009775161743, 541.428584337234497, 292.708322167396545, 87.499996662139893 ],
					"presentation" : 1,
					"presentation_rect" : [ 415.853668451309204, 420.731717348098755, 297.222236394882202, 91.66667103767395 ],
					"viewvisibility" : 1
				}

			}
, 			{
				"box" : 				{
					"bgmode" : 0,
					"border" : 0,
					"clickthrough" : 0,
					"enablehscroll" : 0,
					"enablevscroll" : 0,
					"id" : "obj-18",
					"lockeddragscroll" : 0,
					"lockedsize" : 0,
					"maxclass" : "bpatcher",
					"name" : "MCHelper.maxpat",
					"numinlets" : 1,
					"numoutlets" : 2,
					"offset" : [ 0.0, 0.0 ],
					"outlettype" : [ "", "int" ],
					"patching_rect" : [ 415.714295625686646, 364.285722970962524, 235.802487969398499, 158.024703979492188 ],
					"presentation" : 1,
					"presentation_rect" : [ 415.853668451309204, 248.78049373626709, 237.500011324882507, 175.000008344650269 ],
					"viewvisibility" : 1
				}

			}
, 			{
				"box" : 				{
					"args" : [ "tenor" ],
					"bgmode" : 0,
					"border" : 0,
					"clickthrough" : 0,
					"enablehscroll" : 0,
					"enablevscroll" : 0,
					"id" : "obj-19",
					"lockeddragscroll" : 0,
					"lockedsize" : 0,
					"maxclass" : "bpatcher",
					"name" : "MCModule.maxpat",
					"numinlets" : 3,
					"numoutlets" : 2,
					"offset" : [ 0.0, 0.0 ],
					"outlettype" : [ "signal", "signal" ],
					"patching_rect" : [ -11.428571701049805, 352.85715126991272, 410.975619554519653, 323.170739412307739 ],
					"presentation" : 1,
					"presentation_rect" : [ 10.975610017776489, 259.756103754043579, 398.412704586982727, 260.317464351654053 ],
					"viewvisibility" : 1
				}

			}
, 			{
				"box" : 				{
					"args" : [ "alto" ],
					"bgmode" : 0,
					"border" : 0,
					"clickthrough" : 0,
					"enablehscroll" : 0,
					"enablevscroll" : 0,
					"id" : "obj-8",
					"lockeddragscroll" : 0,
					"lockedsize" : 0,
					"maxclass" : "bpatcher",
					"name" : "gridmodule.maxpat",
					"numinlets" : 0,
					"numoutlets" : 2,
					"offset" : [ 0.0, 0.0 ],
					"outlettype" : [ "", "" ],
					"patching_rect" : [ 725.00003457069397, 757.142875194549561, 329.333343148231506, 222.290322661399841 ],
					"presentation" : 1,
					"presentation_rect" : [ 717.073187828063965, 539.024403095245361, 329.333343148231506, 222.290322661399841 ],
					"viewvisibility" : 1
				}

			}
, 			{
				"box" : 				{
					"bgmode" : 0,
					"border" : 0,
					"clickthrough" : 0,
					"enablehscroll" : 0,
					"enablevscroll" : 0,
					"id" : "obj-12",
					"lockeddragscroll" : 0,
					"lockedsize" : 0,
					"maxclass" : "bpatcher",
					"name" : "MCLyra.maxpat",
					"numinlets" : 2,
					"numoutlets" : 1,
					"offset" : [ 0.0, 0.0 ],
					"outlettype" : [ "multichannelsignal" ],
					"patching_rect" : [ 422.22224235534668, 911.111154556274414, 292.708322167396545, 87.499996662139893 ],
					"presentation" : 1,
					"presentation_rect" : [ 415.853668451309204, 674.390259981155396, 297.222236394882202, 91.66667103767395 ],
					"viewvisibility" : 1
				}

			}
, 			{
				"box" : 				{
					"bgmode" : 0,
					"border" : 0,
					"clickthrough" : 0,
					"enablehscroll" : 0,
					"enablevscroll" : 0,
					"id" : "obj-13",
					"lockeddragscroll" : 0,
					"lockedsize" : 0,
					"maxclass" : "bpatcher",
					"name" : "MCHelper.maxpat",
					"numinlets" : 1,
					"numoutlets" : 2,
					"offset" : [ 0.0, 0.0 ],
					"outlettype" : [ "", "int" ],
					"patching_rect" : [ 427.777798175811768, 733.333368301391602, 235.802487969398499, 158.024703979492188 ],
					"presentation" : 1,
					"presentation_rect" : [ 415.853668451309204, 503.658548593521118, 237.500011324882507, 175.000008344650269 ],
					"viewvisibility" : 1
				}

			}
, 			{
				"box" : 				{
					"args" : [ "alto" ],
					"bgmode" : 0,
					"border" : 0,
					"clickthrough" : 0,
					"enablehscroll" : 0,
					"enablevscroll" : 0,
					"id" : "obj-14",
					"lockeddragscroll" : 0,
					"lockedsize" : 0,
					"maxclass" : "bpatcher",
					"name" : "MCModule.maxpat",
					"numinlets" : 3,
					"numoutlets" : 2,
					"offset" : [ 0.0, 0.0 ],
					"outlettype" : [ "signal", "signal" ],
					"patching_rect" : [ -26.388890147209167, 702.777811288833618, 410.975619554519653, 323.170739412307739 ],
					"presentation" : 1,
					"presentation_rect" : [ 10.975610017776489, 514.634158611297607, 398.412704586982727, 260.317464351654053 ],
					"viewvisibility" : 1
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-5",
					"maxclass" : "comment",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 505.555579662322998, 683.333365917205811, 150.0, 20.0 ],
					"text" : "38.7 sounds dope"
				}

			}
, 			{
				"box" : 				{
					"args" : [ "soprano" ],
					"bgmode" : 0,
					"border" : 0,
					"clickthrough" : 0,
					"enablehscroll" : 0,
					"enablevscroll" : 0,
					"id" : "obj-2",
					"lockeddragscroll" : 0,
					"lockedsize" : 0,
					"maxclass" : "bpatcher",
					"name" : "gridmodule.maxpat",
					"numinlets" : 0,
					"numoutlets" : 2,
					"offset" : [ 0.0, 0.0 ],
					"outlettype" : [ "", "" ],
					"patching_rect" : [ 715.116656851768539, 1067.142882585525513, 329.333343148231506, 222.290322661399841 ],
					"presentation" : 1,
					"presentation_rect" : [ 717.073187828063965, 793.90245795249939, 329.333343148231506, 222.290322661399841 ],
					"viewvisibility" : 1
				}

			}
, 			{
				"box" : 				{
					"bgmode" : 0,
					"border" : 0,
					"clickthrough" : 0,
					"enablehscroll" : 0,
					"enablevscroll" : 0,
					"id" : "obj-1",
					"lockeddragscroll" : 0,
					"lockedsize" : 0,
					"maxclass" : "bpatcher",
					"name" : "MCLyra.maxpat",
					"numinlets" : 2,
					"numoutlets" : 1,
					"offset" : [ 0.0, 0.0 ],
					"outlettype" : [ "multichannelsignal" ],
					"patching_rect" : [ 409.722241759300232, 1195.833390355110168, 292.708322167396545, 87.499996662139893 ],
					"presentation" : 1,
					"presentation_rect" : [ 415.853668451309204, 931.707339286804199, 297.222236394882202, 91.66667103767395 ],
					"viewvisibility" : 1
				}

			}
, 			{
				"box" : 				{
					"args" : [ "bass" ],
					"bgmode" : 0,
					"border" : 0,
					"clickthrough" : 0,
					"enablehscroll" : 0,
					"enablevscroll" : 0,
					"id" : "obj-3",
					"lockeddragscroll" : 0,
					"lockedsize" : 0,
					"maxclass" : "bpatcher",
					"name" : "gridmodule.maxpat",
					"numinlets" : 0,
					"numoutlets" : 2,
					"offset" : [ 0.0, 0.0 ],
					"outlettype" : [ "", "" ],
					"patching_rect" : [ 728.571445941925049, 18.292683362960815, 329.333343148231506, 222.290322661399841 ],
					"presentation" : 1,
					"presentation_rect" : [ 717.460328578948975, 33.521507740020752, 329.333343148231506, 222.290322661399841 ],
					"viewvisibility" : 1
				}

			}
, 			{
				"box" : 				{
					"bgmode" : 0,
					"border" : 0,
					"clickthrough" : 0,
					"enablehscroll" : 0,
					"enablevscroll" : 0,
					"id" : "obj-15",
					"lockeddragscroll" : 0,
					"lockedsize" : 0,
					"maxclass" : "bpatcher",
					"name" : "MCLyra.maxpat",
					"numinlets" : 2,
					"numoutlets" : 1,
					"offset" : [ 0.0, 0.0 ],
					"outlettype" : [ "multichannelsignal" ],
					"patching_rect" : [ 427.892303586006165, 215.714290857315063, 292.708322167396545, 87.499996662139893 ],
					"presentation" : 1,
					"presentation_rect" : [ 415.972242057323456, 174.999999761581421, 297.222236394882202, 91.66667103767395 ],
					"viewvisibility" : 1
				}

			}
, 			{
				"box" : 				{
					"bgmode" : 0,
					"border" : 0,
					"clickthrough" : 0,
					"enablehscroll" : 0,
					"enablevscroll" : 0,
					"id" : "obj-10",
					"lockeddragscroll" : 0,
					"lockedsize" : 0,
					"maxclass" : "bpatcher",
					"name" : "MCHelper.maxpat",
					"numinlets" : 1,
					"numoutlets" : 2,
					"offset" : [ 0.0, 0.0 ],
					"outlettype" : [ "", "int" ],
					"patching_rect" : [ 415.27779757976532, 1019.444493055343628, 235.802487969398499, 158.024703979492188 ],
					"presentation" : 1,
					"presentation_rect" : [ 415.853668451309204, 754.878066778182983, 236.111122369766235, 175.000008344650269 ],
					"viewvisibility" : 1
				}

			}
, 			{
				"box" : 				{
					"args" : [ "soprano" ],
					"bgmode" : 0,
					"border" : 0,
					"clickthrough" : 0,
					"enablehscroll" : 0,
					"enablevscroll" : 0,
					"id" : "obj-11",
					"lockeddragscroll" : 0,
					"lockedsize" : 0,
					"maxclass" : "bpatcher",
					"name" : "MCModule.maxpat",
					"numinlets" : 3,
					"numoutlets" : 2,
					"offset" : [ 0.0, 0.0 ],
					"outlettype" : [ "signal", "signal" ],
					"patching_rect" : [ -18.055556416511536, 1011.111159324645996, 410.975619554519653, 323.170739412307739 ],
					"presentation" : 1,
					"presentation_rect" : [ 10.975610017776489, 768.292701244354248, 398.412704586982727, 258.730162739753723 ],
					"viewvisibility" : 1
				}

			}
, 			{
				"box" : 				{
					"bgmode" : 0,
					"border" : 0,
					"clickthrough" : 0,
					"enablehscroll" : 0,
					"enablevscroll" : 0,
					"id" : "obj-9",
					"lockeddragscroll" : 0,
					"lockedsize" : 0,
					"maxclass" : "bpatcher",
					"name" : "MCHelper.maxpat",
					"numinlets" : 1,
					"numoutlets" : 2,
					"offset" : [ 0.0, 0.0 ],
					"outlettype" : [ "", "int" ],
					"patching_rect" : [ 427.892303586006165, 18.292683362960815, 266.129034161567688, 179.032259345054626 ],
					"presentation" : 1,
					"presentation_rect" : [ 415.972242057323456, 6.94444477558136, 236.111122369766235, 175.000008344650269 ],
					"viewvisibility" : 1
				}

			}
, 			{
				"box" : 				{
					"args" : [ "bass" ],
					"bgmode" : 0,
					"border" : 0,
					"clickthrough" : 0,
					"enablehscroll" : 0,
					"enablevscroll" : 0,
					"id" : "obj-7",
					"lockeddragscroll" : 0,
					"lockedsize" : 0,
					"maxclass" : "bpatcher",
					"name" : "MCModule.maxpat",
					"numinlets" : 3,
					"numoutlets" : 2,
					"offset" : [ 0.0, 0.0 ],
					"outlettype" : [ "signal", "signal" ],
					"patching_rect" : [ 2.439024448394775, 18.292683362960815, 410.975619554519653, 323.170739412307739 ],
					"presentation" : 1,
					"presentation_rect" : [ 11.111111283302307, 6.349206447601318, 398.412704586982727, 260.317464351654053 ],
					"viewvisibility" : 1
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-68",
					"maxclass" : "panel",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 1266.099999999999909, 381.339999999999975, 128.0, 128.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 20.731707811355591, 23.170732259750366, 1040.243927240371704, 243.902444839477539 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-69",
					"maxclass" : "panel",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 1281.099999999999909, 396.339999999999975, 128.0, 128.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 20.731707811355591, 275.609762668609619, 1040.243927240371704, 243.902444839477539 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-70",
					"maxclass" : "panel",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 1296.099999999999909, 411.339999999999975, 128.0, 128.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 19.512195587158203, 530.487817525863647, 1040.243927240371704, 243.902444839477539 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-71",
					"maxclass" : "panel",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 1311.099999999999909, 426.339999999999975, 128.0, 128.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 19.512195587158203, 782.9268479347229, 1040.243927240371704, 243.902444839477539 ]
				}

			}
 ],
		"lines" : [ 			{
				"patchline" : 				{
					"destination" : [ "obj-11", 2 ],
					"source" : [ "obj-1", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-11", 1 ],
					"source" : [ "obj-10", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-11", 0 ],
					"source" : [ "obj-10", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-101", 1 ],
					"midpoints" : [ 1848.071472406387329, 1590.0, 1746.0, 1590.0, 1746.0, 1458.0, 1817.564449977874801, 1458.0 ],
					"order" : 1,
					"source" : [ "obj-100", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-109", 4 ],
					"midpoints" : [ 1848.071472406387329, 1728.0, 2015.013382434844971, 1728.0 ],
					"order" : 0,
					"source" : [ "obj-100", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-100", 0 ],
					"midpoints" : [ 1850.059366067250494, 1527.0, 1848.071472406387329, 1527.0 ],
					"source" : [ "obj-101", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-103", 0 ],
					"midpoints" : [ 1768.822075843811035, 1527.0, 1769.500041961669922, 1527.0 ],
					"source" : [ "obj-101", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-98", 0 ],
					"midpoints" : [ 2012.533946514129639, 1527.0, 2005.214333295822144, 1527.0 ],
					"source" : [ "obj-101", 3 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-99", 0 ],
					"midpoints" : [ 1931.29665629069018, 1527.0, 1926.642902851104736, 1527.0 ],
					"source" : [ "obj-101", 2 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-115", 7 ],
					"midpoints" : [ 2989.500071048736572, 1713.0, 2418.483887672424316, 1713.0 ],
					"order" : 1,
					"source" : [ "obj-102", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-87", 3 ],
					"midpoints" : [ 2989.500071048736572, 1581.0, 3018.0, 1581.0, 3018.0, 1458.0, 2901.167862033843448, 1458.0 ],
					"order" : 0,
					"source" : [ "obj-102", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-101", 0 ],
					"midpoints" : [ 1769.500041961669922, 1581.0, 1746.0, 1581.0, 1746.0, 1467.0, 1768.822075843811035, 1467.0 ],
					"order" : 1,
					"source" : [ "obj-103", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-72", 4 ],
					"midpoints" : [ 1769.500041961669922, 1737.0, 1827.306464314460754, 1737.0 ],
					"order" : 0,
					"source" : [ "obj-103", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-104", 0 ],
					"source" : [ "obj-106", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-108", 1 ],
					"order" : 0,
					"source" : [ "obj-107", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-117", 0 ],
					"order" : 1,
					"source" : [ "obj-107", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-107", 0 ],
					"source" : [ "obj-109", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-10", 0 ],
					"source" : [ "obj-11", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-111", 1 ],
					"order" : 0,
					"source" : [ "obj-110", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-118", 0 ],
					"order" : 1,
					"source" : [ "obj-110", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-110", 0 ],
					"source" : [ "obj-112", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-114", 1 ],
					"order" : 0,
					"source" : [ "obj-113", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-119", 0 ],
					"order" : 1,
					"source" : [ "obj-113", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-113", 0 ],
					"source" : [ "obj-115", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-14", 2 ],
					"source" : [ "obj-12", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-22", 6 ],
					"midpoints" : [ 2232.0, 504.0, 2292.0, 504.0, 2292.0, 435.0, 2142.0, 435.0 ],
					"source" : [ "obj-120", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-22", 4 ],
					"midpoints" : [ 2283.5, 426.0, 2124.5, 426.0 ],
					"source" : [ "obj-121", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-22", 3 ],
					"midpoints" : [ 2249.5, 426.0, 2115.75, 426.0 ],
					"source" : [ "obj-123", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-22", 2 ],
					"midpoints" : [ 2195.5, 426.0, 2107.0, 426.0 ],
					"source" : [ "obj-124", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-22", 1 ],
					"midpoints" : [ 2165.5, 426.0, 2098.25, 426.0 ],
					"source" : [ "obj-125", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-22", 0 ],
					"midpoints" : [ 2089.5, 417.0, 2089.5, 417.0 ],
					"source" : [ "obj-126", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-22", 5 ],
					"midpoints" : [ 2130.0, 504.0, 2067.0, 504.0, 2067.0, 426.0, 2133.25, 426.0 ],
					"source" : [ "obj-127", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-20", 0 ],
					"midpoints" : [ 2051.369999999999891, 906.0, 2197.32601523399353, 906.0 ],
					"source" : [ "obj-128", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-33", 0 ],
					"midpoints" : [ 2073.869999999999891, 1071.0, 2197.304930210113525, 1071.0 ],
					"source" : [ "obj-128", 3 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-35", 0 ],
					"midpoints" : [ 2062.619999999999891, 990.0, 2197.304930210113525, 990.0 ],
					"source" : [ "obj-128", 2 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-37", 0 ],
					"midpoints" : [ 2040.119999999999891, 825.0, 2197.5, 825.0 ],
					"source" : [ "obj-128", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-14", 1 ],
					"source" : [ "obj-13", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-14", 0 ],
					"source" : [ "obj-13", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-131", 0 ],
					"source" : [ "obj-130", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-130", 0 ],
					"order" : 0,
					"source" : [ "obj-131", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-137", 0 ],
					"order" : 1,
					"source" : [ "obj-131", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-133", 0 ],
					"source" : [ "obj-132", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-132", 0 ],
					"order" : 0,
					"source" : [ "obj-133", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-138", 0 ],
					"order" : 1,
					"source" : [ "obj-133", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-140", 0 ],
					"order" : 1,
					"source" : [ "obj-134", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-219", 0 ],
					"order" : 0,
					"source" : [ "obj-134", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-13", 0 ],
					"source" : [ "obj-14", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-7", 2 ],
					"source" : [ "obj-15", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-17", 0 ],
					"source" : [ "obj-16", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-17", 1 ],
					"source" : [ "obj-16", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-19", 2 ],
					"source" : [ "obj-17", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-19", 1 ],
					"source" : [ "obj-18", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-19", 0 ],
					"source" : [ "obj-18", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-210", 0 ],
					"midpoints" : [ 1770.928613424301147, 1365.0, 1746.0, 1365.0, 1746.0, 1248.0, 1770.516991138458252, 1248.0 ],
					"order" : 1,
					"source" : [ "obj-181", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-72", 0 ],
					"midpoints" : [ 1770.928613424301147, 1458.0, 1746.0, 1458.0, 1746.0, 1737.0, 1785.306464314460754, 1737.0 ],
					"order" : 0,
					"source" : [ "obj-181", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-18", 0 ],
					"source" : [ "obj-19", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-1", 0 ],
					"source" : [ "obj-2", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-1", 1 ],
					"source" : [ "obj-2", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-264", 0 ],
					"midpoints" : [ 2197.32601523399353, 936.0, 2184.0, 936.0, 2184.0, 906.0, 2229.0, 906.0, 2229.0, 915.0, 2243.646394729614258, 915.0 ],
					"order" : 0,
					"source" : [ "obj-20", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-42", 0 ],
					"midpoints" : [ 2197.32601523399353, 936.0, 2199.934710800647736, 936.0 ],
					"order" : 1,
					"source" : [ "obj-20", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-181", 0 ],
					"midpoints" : [ 1770.516991138458252, 1305.0, 1770.928613424301147, 1305.0 ],
					"source" : [ "obj-210", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-212", 0 ],
					"midpoints" : [ 1851.754281361897711, 1311.0, 1848.071472406387329, 1311.0 ],
					"source" : [ "obj-210", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-213", 0 ],
					"midpoints" : [ 1932.991571585337397, 1305.0, 1926.642902851104736, 1305.0 ],
					"source" : [ "obj-210", 2 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-47", 0 ],
					"midpoints" : [ 2014.228861808776855, 1305.0, 2007.0, 1305.0, 2007.0, 1311.0, 2005.214333295822144, 1311.0 ],
					"source" : [ "obj-210", 3 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-109", 0 ],
					"midpoints" : [ 1848.071472406387329, 1458.0, 2034.0, 1458.0, 2034.0, 1728.0, 1973.013382434844971, 1728.0 ],
					"order" : 0,
					"source" : [ "obj-212", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-210", 1 ],
					"midpoints" : [ 1848.071472406387329, 1374.0, 1746.0, 1374.0, 1746.0, 1236.0, 1819.259365272522018, 1236.0 ],
					"order" : 1,
					"source" : [ "obj-212", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-112", 0 ],
					"midpoints" : [ 1926.642902851104736, 1458.0, 2073.0, 1458.0, 2073.0, 1722.0, 2151.43549919128418, 1722.0 ],
					"order" : 0,
					"source" : [ "obj-213", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-210", 2 ],
					"midpoints" : [ 1926.642902851104736, 1374.0, 1746.0, 1374.0, 1746.0, 1236.0, 1868.001739406585784, 1236.0 ],
					"order" : 1,
					"source" : [ "obj-213", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-4", 0 ],
					"source" : [ "obj-216", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-134", 0 ],
					"source" : [ "obj-219", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-120", 0 ],
					"midpoints" : [ 2142.0, 468.0, 2191.5, 468.0 ],
					"source" : [ "obj-22", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-127", 0 ],
					"midpoints" : [ 2089.5, 462.0, 2089.5, 462.0 ],
					"source" : [ "obj-22", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-21", 1 ],
					"source" : [ "obj-23", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-21", 0 ],
					"source" : [ "obj-23", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-33", 0 ],
					"midpoints" : [ 2243.646394729614258, 1161.0, 2172.0, 1161.0, 2172.0, 1080.0, 2197.304930210113525, 1080.0 ],
					"source" : [ "obj-252", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-35", 0 ],
					"midpoints" : [ 2243.646394729614258, 1080.0, 2220.0, 1080.0, 2220.0, 1053.0, 2172.0, 1053.0, 2172.0, 999.0, 2197.304930210113525, 999.0 ],
					"source" : [ "obj-258", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-20", 0 ],
					"midpoints" : [ 2243.646394729614258, 999.0, 2220.0, 999.0, 2220.0, 972.0, 2172.0, 972.0, 2172.0, 915.0, 2197.32601523399353, 915.0 ],
					"source" : [ "obj-264", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-37", 0 ],
					"midpoints" : [ 2243.646394729614258, 918.0, 2229.0, 918.0, 2229.0, 906.0, 2175.0, 906.0, 2175.0, 837.0, 2197.5, 837.0 ],
					"source" : [ "obj-270", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-15", 0 ],
					"source" : [ "obj-3", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-15", 1 ],
					"source" : [ "obj-3", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-252", 0 ],
					"midpoints" : [ 2197.304930210113525, 1101.0, 2184.0, 1101.0, 2184.0, 1071.0, 2229.0, 1071.0, 2229.0, 1080.0, 2243.646394729614258, 1080.0 ],
					"order" : 0,
					"source" : [ "obj-33", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-46", 0 ],
					"midpoints" : [ 2197.304930210113525, 1101.0, 2198.524442434310913, 1101.0 ],
					"order" : 1,
					"source" : [ "obj-33", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-258", 0 ],
					"midpoints" : [ 2197.304930210113525, 1020.0, 2184.0, 1020.0, 2184.0, 990.0, 2229.0, 990.0, 2229.0, 996.0, 2243.646394729614258, 996.0 ],
					"order" : 0,
					"source" : [ "obj-35", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-44", 0 ],
					"midpoints" : [ 2197.304930210113525, 1020.0, 2197.304930210113525, 1020.0 ],
					"order" : 1,
					"source" : [ "obj-35", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-270", 0 ],
					"midpoints" : [ 2197.5, 855.0, 2175.0, 855.0, 2175.0, 825.0, 2243.646394729614258, 825.0 ],
					"order" : 0,
					"source" : [ "obj-37", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-39", 0 ],
					"midpoints" : [ 2197.5, 858.0, 2197.5, 858.0 ],
					"order" : 1,
					"source" : [ "obj-37", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-58", 0 ],
					"midpoints" : [ 2197.5, 906.0, 2170.928622961044312, 906.0 ],
					"source" : [ "obj-39", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-139", 0 ],
					"order" : 1,
					"source" : [ "obj-4", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-216", 0 ],
					"order" : 0,
					"source" : [ "obj-4", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-128", 0 ],
					"source" : [ "obj-40", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-58", 0 ],
					"midpoints" : [ 2199.934710800647736, 990.0, 2170.928622961044312, 990.0 ],
					"source" : [ "obj-42", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-58", 0 ],
					"midpoints" : [ 2197.304930210113525, 1071.0, 2170.928622961044312, 1071.0 ],
					"source" : [ "obj-44", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-58", 0 ],
					"midpoints" : [ 2198.524442434310913, 1197.0, 2170.928622961044312, 1197.0 ],
					"source" : [ "obj-46", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-115", 0 ],
					"midpoints" : [ 2005.214333295822144, 1458.0, 2073.0, 1458.0, 2073.0, 1713.0, 2344.983887672424316, 1713.0 ],
					"order" : 0,
					"source" : [ "obj-47", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-210", 3 ],
					"midpoints" : [ 2005.214333295822144, 1365.0, 2034.0, 1365.0, 2034.0, 1203.0, 1916.744113540649323, 1203.0 ],
					"order" : 1,
					"source" : [ "obj-47", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-115", 1 ],
					"midpoints" : [ 2336.642912626266479, 1458.0, 2361.0, 1458.0, 2361.0, 1713.0, 2355.483887672424316, 1713.0 ],
					"order" : 0,
					"source" : [ "obj-48", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-52", 3 ],
					"midpoints" : [ 2336.642912626266479, 1365.0, 2364.0, 1365.0, 2364.0, 1236.0, 2247.252595996856599, 1236.0 ],
					"order" : 1,
					"source" : [ "obj-48", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-112", 1 ],
					"midpoints" : [ 2258.071482181549072, 1458.0, 2073.0, 1458.0, 2073.0, 1722.0, 2161.93549919128418, 1722.0 ],
					"order" : 1,
					"source" : [ "obj-49", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-52", 2 ],
					"midpoints" : [ 2258.071482181549072, 1374.0, 2364.0, 1374.0, 2364.0, 1236.0, 2199.0, 1236.0, 2199.0, 1248.0, 2198.51022186279306, 1248.0 ],
					"order" : 0,
					"source" : [ "obj-49", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-109", 1 ],
					"midpoints" : [ 2179.500051736831665, 1458.0, 2034.0, 1458.0, 2034.0, 1728.0, 1983.513382434844971, 1728.0 ],
					"order" : 1,
					"source" : [ "obj-50", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-52", 1 ],
					"midpoints" : [ 2179.500051736831665, 1374.0, 2076.0, 1374.0, 2076.0, 1236.0, 2149.767847728729066, 1236.0 ],
					"order" : 0,
					"source" : [ "obj-50", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-52", 0 ],
					"midpoints" : [ 2100.928621292114258, 1365.0, 2076.0, 1365.0, 2076.0, 1248.0, 2101.025473594665527, 1248.0 ],
					"order" : 0,
					"source" : [ "obj-51", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-72", 1 ],
					"midpoints" : [ 2100.928621292114258, 1458.0, 2034.0, 1458.0, 2034.0, 1728.0, 1795.806464314460754, 1728.0 ],
					"order" : 1,
					"source" : [ "obj-51", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-48", 0 ],
					"midpoints" : [ 2344.737344264984131, 1305.0, 2337.0, 1305.0, 2337.0, 1311.0, 2336.642912626266479, 1311.0 ],
					"source" : [ "obj-52", 3 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-49", 0 ],
					"midpoints" : [ 2263.500054041544445, 1305.0, 2258.071482181549072, 1305.0 ],
					"source" : [ "obj-52", 2 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-50", 0 ],
					"midpoints" : [ 2182.262763818105213, 1311.0, 2179.500051736831665, 1311.0 ],
					"source" : [ "obj-52", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-51", 0 ],
					"midpoints" : [ 2101.025473594665527, 1305.0, 2100.928621292114258, 1305.0 ],
					"source" : [ "obj-52", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-101", 4 ],
					"midpoints" : [ 2170.928622961044312, 1236.0, 2034.0, 1236.0, 2034.0, 1458.0, 1963.791572380065872, 1458.0 ],
					"order" : 7,
					"source" : [ "obj-58", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-210", 4 ],
					"midpoints" : [ 2170.928622961044312, 1236.0, 1989.0, 1236.0, 1989.0, 1245.0, 1965.486487674713089, 1245.0 ],
					"order" : 6,
					"source" : [ "obj-58", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-52", 4 ],
					"midpoints" : [ 2170.928622961044312, 1236.0, 2295.994970130920592, 1236.0 ],
					"order" : 4,
					"source" : [ "obj-58", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-67", 4 ],
					"midpoints" : [ 2170.928622961044312, 1236.0, 2626.503452587127867, 1236.0 ],
					"order" : 2,
					"source" : [ "obj-58", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-82", 4 ],
					"midpoints" : [ 2170.928622961044312, 1236.0, 2957.011935043335143, 1236.0 ],
					"order" : 0,
					"source" : [ "obj-58", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-87", 4 ],
					"midpoints" : [ 2170.928622961044312, 1236.0, 2739.0, 1236.0, 2739.0, 1458.0, 2950.367862033843721, 1458.0 ],
					"order" : 1,
					"source" : [ "obj-58", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-92", 4 ],
					"midpoints" : [ 2170.928622961044312, 1236.0, 2406.0, 1236.0, 2406.0, 1458.0, 2620.288763173421103, 1458.0 ],
					"order" : 3,
					"source" : [ "obj-58", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-97", 4 ],
					"midpoints" : [ 2170.928622961044312, 1236.0, 2364.0, 1236.0, 2364.0, 1458.0, 2292.040167776743601, 1458.0 ],
					"order" : 5,
					"source" : [ "obj-58", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-115", 2 ],
					"midpoints" : [ 2666.64292049407959, 1458.0, 2365.983887672424316, 1458.0 ],
					"order" : 1,
					"source" : [ "obj-63", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-67", 3 ],
					"midpoints" : [ 2666.64292049407959, 1365.0, 2694.0, 1365.0, 2694.0, 1236.0, 2577.761078453063874, 1236.0 ],
					"order" : 0,
					"source" : [ "obj-63", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-112", 2 ],
					"midpoints" : [ 2588.071490049362183, 1458.0, 2361.0, 1458.0, 2361.0, 1713.0, 2172.43549919128418, 1713.0 ],
					"order" : 1,
					"source" : [ "obj-64", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-67", 2 ],
					"midpoints" : [ 2588.071490049362183, 1374.0, 2694.0, 1374.0, 2694.0, 1236.0, 2529.018704319000335, 1236.0 ],
					"order" : 0,
					"source" : [ "obj-64", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-109", 2 ],
					"midpoints" : [ 2508.07148814201355, 1458.0, 2034.0, 1458.0, 2034.0, 1728.0, 1994.013382434844971, 1728.0 ],
					"order" : 1,
					"source" : [ "obj-65", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-67", 1 ],
					"midpoints" : [ 2508.07148814201355, 1374.0, 2406.0, 1374.0, 2406.0, 1236.0, 2480.276330184936342, 1236.0 ],
					"order" : 0,
					"source" : [ "obj-65", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-67", 0 ],
					"midpoints" : [ 2429.500057697296143, 1365.0, 2406.0, 1365.0, 2406.0, 1248.0, 2431.533956050872803, 1248.0 ],
					"order" : 0,
					"source" : [ "obj-66", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-72", 2 ],
					"midpoints" : [ 2429.500057697296143, 1458.0, 2034.0, 1458.0, 2034.0, 1728.0, 1806.306464314460754, 1728.0 ],
					"order" : 1,
					"source" : [ "obj-66", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-63", 0 ],
					"midpoints" : [ 2675.245826721191406, 1305.0, 2667.0, 1305.0, 2667.0, 1311.0, 2666.64292049407959, 1311.0 ],
					"source" : [ "obj-67", 3 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-64", 0 ],
					"midpoints" : [ 2594.00853649775172, 1305.0, 2588.071490049362183, 1305.0 ],
					"source" : [ "obj-67", 2 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-65", 0 ],
					"midpoints" : [ 2512.771246274312489, 1311.0, 2508.07148814201355, 1311.0 ],
					"source" : [ "obj-67", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-66", 0 ],
					"midpoints" : [ 2431.533956050872803, 1311.0, 2429.500057697296143, 1311.0 ],
					"source" : [ "obj-67", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-23", 1 ],
					"source" : [ "obj-7", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-9", 0 ],
					"source" : [ "obj-7", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-83", 0 ],
					"source" : [ "obj-72", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-101", 5 ],
					"midpoints" : [ 1973.013382434844971, 1242.0, 2034.0, 1242.0, 2034.0, 1458.0, 2012.533946514129639, 1458.0 ],
					"order" : 7,
					"source" : [ "obj-77", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-210", 5 ],
					"midpoints" : [ 1973.013382434844971, 1248.0, 2014.228861808776855, 1248.0 ],
					"order" : 6,
					"source" : [ "obj-77", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-52", 5 ],
					"midpoints" : [ 1973.013382434844971, 1242.0, 2076.0, 1242.0, 2076.0, 1236.0, 2344.737344264984131, 1236.0 ],
					"order" : 4,
					"source" : [ "obj-77", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-67", 5 ],
					"midpoints" : [ 1973.013382434844971, 1242.0, 2076.0, 1242.0, 2076.0, 1236.0, 2675.245826721191406, 1236.0 ],
					"order" : 2,
					"source" : [ "obj-77", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-82", 5 ],
					"order" : 0,
					"source" : [ "obj-77", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-87", 5 ],
					"midpoints" : [ 1973.013382434844971, 1242.0, 2076.0, 1242.0, 2076.0, 1458.0, 2999.567862033843539, 1458.0 ],
					"order" : 1,
					"source" : [ "obj-77", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-92", 5 ],
					"midpoints" : [ 1973.013382434844971, 1242.0, 2076.0, 1242.0, 2076.0, 1458.0, 2669.031137307484641, 1458.0 ],
					"order" : 3,
					"source" : [ "obj-77", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-97", 5 ],
					"midpoints" : [ 1973.013382434844971, 1242.0, 2076.0, 1242.0, 2076.0, 1458.0, 2340.78254191080714, 1458.0 ],
					"order" : 5,
					"source" : [ "obj-77", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-115", 3 ],
					"midpoints" : [ 3000.928642749786377, 1458.0, 2376.483887672424316, 1458.0 ],
					"order" : 1,
					"source" : [ "obj-78", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-82", 3 ],
					"midpoints" : [ 3000.928642749786377, 1365.0, 3030.0, 1365.0, 3030.0, 1236.0, 2908.269560909271149, 1236.0 ],
					"order" : 0,
					"source" : [ "obj-78", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-112", 3 ],
					"midpoints" : [ 2920.928640842437744, 1458.0, 2361.0, 1458.0, 2361.0, 1713.0, 2182.93549919128418, 1713.0 ],
					"order" : 1,
					"source" : [ "obj-79", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-82", 2 ],
					"midpoints" : [ 2920.928640842437744, 1374.0, 3030.0, 1374.0, 3030.0, 1236.0, 2859.52718677520761, 1236.0 ],
					"order" : 0,
					"source" : [ "obj-79", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-12", 0 ],
					"source" : [ "obj-8", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-12", 1 ],
					"source" : [ "obj-8", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-109", 3 ],
					"midpoints" : [ 2842.357210397720337, 1458.0, 2034.0, 1458.0, 2034.0, 1728.0, 2004.513382434844971, 1728.0 ],
					"order" : 1,
					"source" : [ "obj-80", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-82", 1 ],
					"midpoints" : [ 2842.357210397720337, 1374.0, 2739.0, 1374.0, 2739.0, 1236.0, 2810.784812641143617, 1236.0 ],
					"order" : 0,
					"source" : [ "obj-80", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-72", 3 ],
					"midpoints" : [ 2762.357208490371704, 1458.0, 2034.0, 1458.0, 2034.0, 1728.0, 1816.806464314460754, 1728.0 ],
					"order" : 1,
					"source" : [ "obj-81", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-82", 0 ],
					"midpoints" : [ 2762.357208490371704, 1365.0, 2739.0, 1365.0, 2739.0, 1248.0, 2762.042438507080078, 1248.0 ],
					"order" : 0,
					"source" : [ "obj-81", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-78", 0 ],
					"source" : [ "obj-82", 3 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-79", 0 ],
					"midpoints" : [ 2924.517018953958996, 1311.0, 2920.928640842437744, 1311.0 ],
					"source" : [ "obj-82", 2 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-80", 0 ],
					"midpoints" : [ 2843.279728730519764, 1305.0, 2842.357210397720337, 1305.0 ],
					"source" : [ "obj-82", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-81", 0 ],
					"midpoints" : [ 2762.042438507080078, 1305.0, 2762.357208490371704, 1305.0 ],
					"source" : [ "obj-82", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-116", 0 ],
					"order" : 0,
					"source" : [ "obj-83", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-74", 1 ],
					"order" : 1,
					"source" : [ "obj-83", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-74", 0 ],
					"order" : 2,
					"source" : [ "obj-83", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-112", 7 ],
					"midpoints" : [ 2910.928640604019165, 1713.0, 2224.93549919128418, 1713.0 ],
					"order" : 1,
					"source" : [ "obj-84", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-87", 2 ],
					"midpoints" : [ 2910.928640604019165, 1590.0, 3018.0, 1590.0, 3018.0, 1458.0, 2851.96786203384363, 1458.0 ],
					"order" : 0,
					"source" : [ "obj-84", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-109", 7 ],
					"midpoints" : [ 2832.357210159301758, 1713.0, 2046.513382434844971, 1713.0 ],
					"order" : 1,
					"source" : [ "obj-85", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-87", 1 ],
					"midpoints" : [ 2832.357210159301758, 1590.0, 2730.0, 1590.0, 2730.0, 1458.0, 2802.767862033843357, 1458.0 ],
					"order" : 0,
					"source" : [ "obj-85", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-72", 7 ],
					"midpoints" : [ 2753.785779714584351, 1713.0, 1858.806464314460754, 1713.0 ],
					"order" : 1,
					"source" : [ "obj-86", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-87", 0 ],
					"midpoints" : [ 2753.785779714584351, 1581.0, 2730.0, 1581.0, 2730.0, 1467.0, 2753.567862033843539, 1467.0 ],
					"order" : 0,
					"source" : [ "obj-86", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-102", 0 ],
					"midpoints" : [ 2999.567862033843539, 1527.0, 2989.500071048736572, 1527.0 ],
					"source" : [ "obj-87", 3 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-84", 0 ],
					"midpoints" : [ 2917.567862033843539, 1527.0, 2910.928640604019165, 1527.0 ],
					"source" : [ "obj-87", 2 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-85", 0 ],
					"midpoints" : [ 2835.567862033843539, 1527.0, 2832.357210159301758, 1527.0 ],
					"source" : [ "obj-87", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-86", 0 ],
					"midpoints" : [ 2753.567862033843539, 1527.0, 2753.785779714584351, 1527.0 ],
					"source" : [ "obj-87", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-115", 6 ],
					"midpoints" : [ 2660.928634643554688, 1713.0, 2407.983887672424316, 1713.0 ],
					"order" : 1,
					"source" : [ "obj-88", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-92", 3 ],
					"midpoints" : [ 2660.928634643554688, 1581.0, 2688.0, 1581.0, 2688.0, 1458.0, 2571.546389039357109, 1458.0 ],
					"order" : 0,
					"source" : [ "obj-88", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-112", 6 ],
					"midpoints" : [ 2580.928632736206055, 1713.0, 2214.43549919128418, 1713.0 ],
					"order" : 1,
					"source" : [ "obj-89", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-92", 2 ],
					"midpoints" : [ 2580.928632736206055, 1590.0, 2688.0, 1590.0, 2688.0, 1458.0, 2522.80401490529357, 1458.0 ],
					"order" : 0,
					"source" : [ "obj-89", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-7", 1 ],
					"source" : [ "obj-9", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-7", 0 ],
					"source" : [ "obj-9", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-109", 6 ],
					"midpoints" : [ 2500.928630828857422, 1713.0, 2036.013382434844971, 1713.0 ],
					"order" : 1,
					"source" : [ "obj-90", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-92", 1 ],
					"midpoints" : [ 2500.928630828857422, 1590.0, 2397.0, 1590.0, 2397.0, 1458.0, 2474.061640771229577, 1458.0 ],
					"order" : 0,
					"source" : [ "obj-90", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-72", 6 ],
					"midpoints" : [ 2420.928628921508789, 1713.0, 1848.306464314460754, 1713.0 ],
					"order" : 1,
					"source" : [ "obj-91", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-92", 0 ],
					"midpoints" : [ 2420.928628921508789, 1581.0, 2397.0, 1581.0, 2397.0, 1467.0, 2425.319266637166038, 1467.0 ],
					"order" : 0,
					"source" : [ "obj-91", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-88", 0 ],
					"midpoints" : [ 2669.031137307484641, 1527.0, 2660.928634643554688, 1527.0 ],
					"source" : [ "obj-92", 3 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-89", 0 ],
					"midpoints" : [ 2587.793847084044955, 1527.0, 2580.928632736206055, 1527.0 ],
					"source" : [ "obj-92", 2 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-90", 0 ],
					"midpoints" : [ 2506.556556860605724, 1527.0, 2500.928630828857422, 1527.0 ],
					"source" : [ "obj-92", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-91", 0 ],
					"midpoints" : [ 2425.319266637166038, 1527.0, 2420.928628921508789, 1527.0 ],
					"source" : [ "obj-92", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-115", 5 ],
					"midpoints" : [ 2332.357198238372803, 1713.0, 2397.483887672424316, 1713.0 ],
					"order" : 0,
					"source" : [ "obj-93", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-97", 3 ],
					"midpoints" : [ 2332.357198238372803, 1581.0, 2361.0, 1581.0, 2361.0, 1458.0, 2243.297793642679608, 1458.0 ],
					"order" : 1,
					"source" : [ "obj-93", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-112", 5 ],
					"midpoints" : [ 2253.785767793655396, 1722.0, 2203.93549919128418, 1722.0 ],
					"order" : 0,
					"source" : [ "obj-94", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-97", 2 ],
					"midpoints" : [ 2253.785767793655396, 1590.0, 2361.0, 1590.0, 2361.0, 1458.0, 2194.555419508616069, 1458.0 ],
					"order" : 1,
					"source" : [ "obj-94", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-109", 5 ],
					"midpoints" : [ 2175.214337348937988, 1722.0, 2025.513382434844971, 1722.0 ],
					"order" : 1,
					"source" : [ "obj-95", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-97", 1 ],
					"midpoints" : [ 2175.214337348937988, 1590.0, 2073.0, 1590.0, 2073.0, 1458.0, 2145.813045374552075, 1458.0 ],
					"order" : 0,
					"source" : [ "obj-95", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-72", 5 ],
					"midpoints" : [ 2096.642906904220581, 1728.0, 1837.806464314460754, 1728.0 ],
					"order" : 1,
					"source" : [ "obj-96", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-97", 0 ],
					"midpoints" : [ 2096.642906904220581, 1581.0, 2073.0, 1581.0, 2073.0, 1467.0, 2097.070671240488537, 1467.0 ],
					"order" : 0,
					"source" : [ "obj-96", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-93", 0 ],
					"midpoints" : [ 2340.78254191080714, 1527.0, 2332.357198238372803, 1527.0 ],
					"source" : [ "obj-97", 3 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-94", 0 ],
					"midpoints" : [ 2259.545251687367454, 1527.0, 2253.785767793655396, 1527.0 ],
					"source" : [ "obj-97", 2 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-95", 0 ],
					"midpoints" : [ 2178.307961463928223, 1527.0, 2175.214337348937988, 1527.0 ],
					"source" : [ "obj-97", 1 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-96", 0 ],
					"midpoints" : [ 2097.070671240488537, 1527.0, 2096.642906904220581, 1527.0 ],
					"source" : [ "obj-97", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-101", 3 ],
					"midpoints" : [ 2005.214333295822144, 1581.0, 2034.0, 1581.0, 2034.0, 1458.0, 1915.049198246002106, 1458.0 ],
					"order" : 1,
					"source" : [ "obj-98", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-115", 4 ],
					"midpoints" : [ 2005.214333295822144, 1713.0, 2386.983887672424316, 1713.0 ],
					"order" : 0,
					"source" : [ "obj-98", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-101", 2 ],
					"midpoints" : [ 1926.642902851104736, 1590.0, 2034.0, 1590.0, 2034.0, 1458.0, 1866.306824111938568, 1458.0 ],
					"order" : 1,
					"source" : [ "obj-99", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-112", 4 ],
					"midpoints" : [ 1926.642902851104736, 1722.0, 2193.43549919128418, 1722.0 ],
					"order" : 0,
					"source" : [ "obj-99", 0 ]
				}

			}
 ],
		"parameters" : 		{
			"obj-100" : [ "2[29]", "2", 0 ],
			"obj-102" : [ "2[30]", "2", 0 ],
			"obj-103" : [ "2[31]", "2", 0 ],
			"obj-10::obj-20" : [ "live.menu[3]", "live.menu", 0 ],
			"obj-10::obj-4" : [ "live.dial[3]", "live.dial", 0 ],
			"obj-11::obj-104" : [ "-[21]", "1", 0 ],
			"obj-11::obj-105" : [ "-[25]", "1", 0 ],
			"obj-11::obj-106" : [ "-[22]", "1", 0 ],
			"obj-11::obj-107" : [ "-[30]", "1", 0 ],
			"obj-11::obj-142" : [ "mc.live.gain~[3]", "mc.live.gain~[1]", 0 ],
			"obj-11::obj-15" : [ "button[2]", "button[7]", 0 ],
			"obj-11::obj-159" : [ "live.button[17]", "live.button[1]", 0 ],
			"obj-11::obj-162" : [ "live.button[11]", "live.button[1]", 0 ],
			"obj-11::obj-165" : [ "live.button[12]", "live.button[1]", 0 ],
			"obj-11::obj-168" : [ "button[1]", "button", 0 ],
			"obj-11::obj-172" : [ "live.button[13]", "live.button[1]", 0 ],
			"obj-11::obj-175" : [ "live.button[14]", "live.button[1]", 0 ],
			"obj-11::obj-178" : [ "live.button[15]", "live.button[1]", 0 ],
			"obj-11::obj-181" : [ "live.button[16]", "live.button[1]", 0 ],
			"obj-11::obj-224" : [ "-[17]", "1", 0 ],
			"obj-11::obj-225" : [ "-[16]", "1", 0 ],
			"obj-11::obj-226" : [ "-[24]", "1", 0 ],
			"obj-11::obj-227" : [ "-[29]", "1", 0 ],
			"obj-11::obj-228" : [ "-[26]", "1", 0 ],
			"obj-11::obj-229" : [ "-[28]", "1", 0 ],
			"obj-11::obj-230" : [ "-[31]", "1", 0 ],
			"obj-11::obj-231" : [ "-[23]", "1", 0 ],
			"obj-11::obj-62" : [ "live.menu[2]", "live.menu", 0 ],
			"obj-11::obj-70" : [ "live.button[3]", "live.button[1]", 0 ],
			"obj-11::obj-79" : [ "live.numbox[1]", "live.numbox", 0 ],
			"obj-11::obj-89" : [ "flonum[2]", "flonum", 0 ],
			"obj-11::obj-9" : [ "multislider[1]", "multislider", 0 ],
			"obj-11::obj-90" : [ "number[11]", "number[2]", 0 ],
			"obj-11::obj-91" : [ "number[10]", "number[2]", 0 ],
			"obj-11::obj-92" : [ "number[9]", "number[2]", 0 ],
			"obj-11::obj-96" : [ "-[18]", "1", 0 ],
			"obj-11::obj-97" : [ "-[20]", "1", 0 ],
			"obj-11::obj-98" : [ "-[19]", "1", 0 ],
			"obj-11::obj-99" : [ "-[27]", "1", 0 ],
			"obj-12::obj-159" : [ "live.button[53]", "live.button[1]", 0 ],
			"obj-12::obj-162" : [ "live.button[49]", "live.button[1]", 0 ],
			"obj-12::obj-165" : [ "live.button[46]", "live.button[1]", 0 ],
			"obj-12::obj-172" : [ "live.button[50]", "live.button[1]", 0 ],
			"obj-12::obj-175" : [ "live.button[51]", "live.button[1]", 0 ],
			"obj-12::obj-178" : [ "live.button[47]", "live.button[1]", 0 ],
			"obj-12::obj-181" : [ "live.button[52]", "live.button[1]", 0 ],
			"obj-12::obj-70" : [ "live.button[48]", "live.button[1]", 0 ],
			"obj-12::obj-89" : [ "flonum[6]", "flonum", 0 ],
			"obj-12::obj-90" : [ "number[21]", "number[2]", 0 ],
			"obj-12::obj-91" : [ "number[22]", "number[2]", 0 ],
			"obj-12::obj-92" : [ "number[23]", "number[2]", 0 ],
			"obj-131" : [ "pushtext2[1]", "pushtext2", 0 ],
			"obj-133" : [ "pushtext3[1]", "pushtext3", 0 ],
			"obj-134" : [ "pushtext5[1]", "pushtext5", 0 ],
			"obj-13::obj-20" : [ "live.menu[5]", "live.menu", 0 ],
			"obj-13::obj-4" : [ "live.dial[7]", "live.dial", 0 ],
			"obj-14::obj-104" : [ "-[32]", "1", 0 ],
			"obj-14::obj-105" : [ "-[36]", "1", 0 ],
			"obj-14::obj-106" : [ "-[37]", "1", 0 ],
			"obj-14::obj-107" : [ "-[40]", "1", 0 ],
			"obj-14::obj-142" : [ "mc.live.gain~[4]", "mc.live.gain~[1]", 0 ],
			"obj-14::obj-15" : [ "button[3]", "button[7]", 0 ],
			"obj-14::obj-159" : [ "live.button[43]", "live.button[1]", 0 ],
			"obj-14::obj-162" : [ "live.button[42]", "live.button[1]", 0 ],
			"obj-14::obj-165" : [ "live.button[41]", "live.button[1]", 0 ],
			"obj-14::obj-168" : [ "button[8]", "button", 0 ],
			"obj-14::obj-172" : [ "live.button[39]", "live.button[1]", 0 ],
			"obj-14::obj-175" : [ "live.button[44]", "live.button[1]", 0 ],
			"obj-14::obj-178" : [ "live.button[40]", "live.button[1]", 0 ],
			"obj-14::obj-181" : [ "live.button[38]", "live.button[1]", 0 ],
			"obj-14::obj-224" : [ "-[47]", "1", 0 ],
			"obj-14::obj-225" : [ "-[45]", "1", 0 ],
			"obj-14::obj-226" : [ "-[44]", "1", 0 ],
			"obj-14::obj-227" : [ "-[43]", "1", 0 ],
			"obj-14::obj-228" : [ "-[42]", "1", 0 ],
			"obj-14::obj-229" : [ "-[46]", "1", 0 ],
			"obj-14::obj-230" : [ "-[33]", "1", 0 ],
			"obj-14::obj-231" : [ "-[38]", "1", 0 ],
			"obj-14::obj-62" : [ "live.menu[4]", "live.menu", 0 ],
			"obj-14::obj-70" : [ "live.button[45]", "live.button[1]", 0 ],
			"obj-14::obj-79" : [ "live.numbox[2]", "live.numbox", 0 ],
			"obj-14::obj-89" : [ "flonum[5]", "flonum", 0 ],
			"obj-14::obj-9" : [ "multislider[2]", "multislider", 0 ],
			"obj-14::obj-90" : [ "number[20]", "number[2]", 0 ],
			"obj-14::obj-91" : [ "number[19]", "number[2]", 0 ],
			"obj-14::obj-92" : [ "number[18]", "number[2]", 0 ],
			"obj-14::obj-96" : [ "-[34]", "1", 0 ],
			"obj-14::obj-97" : [ "-[39]", "1", 0 ],
			"obj-14::obj-98" : [ "-[41]", "1", 0 ],
			"obj-14::obj-99" : [ "-[35]", "1", 0 ],
			"obj-15::obj-159" : [ "live.button[22]", "live.button[1]", 0 ],
			"obj-15::obj-162" : [ "live.button[24]", "live.button[1]", 0 ],
			"obj-15::obj-165" : [ "live.button[27]", "live.button[1]", 0 ],
			"obj-15::obj-172" : [ "live.button[29]", "live.button[1]", 0 ],
			"obj-15::obj-175" : [ "live.button[23]", "live.button[1]", 0 ],
			"obj-15::obj-178" : [ "live.button[26]", "live.button[1]", 0 ],
			"obj-15::obj-181" : [ "live.button[25]", "live.button[1]", 0 ],
			"obj-15::obj-70" : [ "live.button[28]", "live.button[1]", 0 ],
			"obj-15::obj-89" : [ "flonum[3]", "flonum", 0 ],
			"obj-15::obj-90" : [ "number[12]", "number[2]", 0 ],
			"obj-15::obj-91" : [ "number[15]", "number[2]", 0 ],
			"obj-15::obj-92" : [ "number[14]", "number[2]", 0 ],
			"obj-16::obj-4" : [ "live.dial[11]", "live.dial", 0 ],
			"obj-16::obj-8" : [ "live.grid[2]", "live.grid", 0 ],
			"obj-17::obj-159" : [ "live.button[64]", "live.button[1]", 0 ],
			"obj-17::obj-162" : [ "live.button[62]", "live.button[1]", 0 ],
			"obj-17::obj-165" : [ "live.button[68]", "live.button[1]", 0 ],
			"obj-17::obj-172" : [ "live.button[66]", "live.button[1]", 0 ],
			"obj-17::obj-175" : [ "live.button[67]", "live.button[1]", 0 ],
			"obj-17::obj-178" : [ "live.button[69]", "live.button[1]", 0 ],
			"obj-17::obj-181" : [ "live.button[63]", "live.button[1]", 0 ],
			"obj-17::obj-70" : [ "live.button[65]", "live.button[1]", 0 ],
			"obj-17::obj-89" : [ "flonum[8]", "flonum", 0 ],
			"obj-17::obj-90" : [ "number[29]", "number[2]", 0 ],
			"obj-17::obj-91" : [ "number[28]", "number[2]", 0 ],
			"obj-17::obj-92" : [ "number[27]", "number[2]", 0 ],
			"obj-181" : [ "2", "2", 0 ],
			"obj-18::obj-20" : [ "live.menu[7]", "live.menu", 0 ],
			"obj-18::obj-4" : [ "live.dial[10]", "live.dial", 0 ],
			"obj-19::obj-104" : [ "-[48]", "1", 0 ],
			"obj-19::obj-105" : [ "-[50]", "1", 0 ],
			"obj-19::obj-106" : [ "-[54]", "1", 0 ],
			"obj-19::obj-107" : [ "-[58]", "1", 0 ],
			"obj-19::obj-142" : [ "mc.live.gain~[5]", "mc.live.gain~[1]", 0 ],
			"obj-19::obj-15" : [ "button[4]", "button[7]", 0 ],
			"obj-19::obj-159" : [ "live.button[61]", "live.button[1]", 0 ],
			"obj-19::obj-162" : [ "live.button[60]", "live.button[1]", 0 ],
			"obj-19::obj-165" : [ "live.button[54]", "live.button[1]", 0 ],
			"obj-19::obj-168" : [ "button[5]", "button", 0 ],
			"obj-19::obj-172" : [ "live.button[55]", "live.button[1]", 0 ],
			"obj-19::obj-175" : [ "live.button[57]", "live.button[1]", 0 ],
			"obj-19::obj-178" : [ "live.button[58]", "live.button[1]", 0 ],
			"obj-19::obj-181" : [ "live.button[59]", "live.button[1]", 0 ],
			"obj-19::obj-224" : [ "-[60]", "1", 0 ],
			"obj-19::obj-225" : [ "-[56]", "1", 0 ],
			"obj-19::obj-226" : [ "-[52]", "1", 0 ],
			"obj-19::obj-227" : [ "-[49]", "1", 0 ],
			"obj-19::obj-228" : [ "-[51]", "1", 0 ],
			"obj-19::obj-229" : [ "-[55]", "1", 0 ],
			"obj-19::obj-230" : [ "-[59]", "1", 0 ],
			"obj-19::obj-231" : [ "-[62]", "1", 0 ],
			"obj-19::obj-62" : [ "live.menu[6]", "live.menu", 0 ],
			"obj-19::obj-70" : [ "live.button[56]", "live.button[1]", 0 ],
			"obj-19::obj-79" : [ "live.numbox[3]", "live.numbox", 0 ],
			"obj-19::obj-89" : [ "flonum[7]", "flonum", 0 ],
			"obj-19::obj-9" : [ "multislider[3]", "multislider", 0 ],
			"obj-19::obj-90" : [ "number[26]", "number[2]", 0 ],
			"obj-19::obj-91" : [ "number[25]", "number[2]", 0 ],
			"obj-19::obj-92" : [ "number[24]", "number[2]", 0 ],
			"obj-19::obj-96" : [ "-[61]", "1", 0 ],
			"obj-19::obj-97" : [ "-[63]", "1", 0 ],
			"obj-19::obj-98" : [ "-[53]", "1", 0 ],
			"obj-19::obj-99" : [ "-[57]", "1", 0 ],
			"obj-1::obj-159" : [ "live.button[35]", "live.button[1]", 0 ],
			"obj-1::obj-162" : [ "live.button[31]", "live.button[1]", 0 ],
			"obj-1::obj-165" : [ "live.button[34]", "live.button[1]", 0 ],
			"obj-1::obj-172" : [ "live.button[32]", "live.button[1]", 0 ],
			"obj-1::obj-175" : [ "live.button[36]", "live.button[1]", 0 ],
			"obj-1::obj-178" : [ "live.button[33]", "live.button[1]", 0 ],
			"obj-1::obj-181" : [ "live.button[30]", "live.button[1]", 0 ],
			"obj-1::obj-70" : [ "live.button[37]", "live.button[1]", 0 ],
			"obj-1::obj-89" : [ "flonum[4]", "flonum", 0 ],
			"obj-1::obj-90" : [ "number[16]", "number[2]", 0 ],
			"obj-1::obj-91" : [ "number[13]", "number[2]", 0 ],
			"obj-1::obj-92" : [ "number[17]", "number[2]", 0 ],
			"obj-20" : [ "pushtext20[1]", "pushtext20", 0 ],
			"obj-212" : [ "2[1]", "2", 0 ],
			"obj-213" : [ "2[2]", "2", 0 ],
			"obj-23" : [ "live.gain~", "live.gain~", 0 ],
			"obj-2::obj-4" : [ "live.dial[5]", "live.dial", 0 ],
			"obj-2::obj-8" : [ "live.grid[1]", "live.grid", 0 ],
			"obj-33" : [ "pushtext16[1]", "pushtext16", 0 ],
			"obj-35" : [ "pushtext18[1]", "pushtext18", 0 ],
			"obj-37" : [ "pushtext22[1]", "pushtext22", 0 ],
			"obj-3::obj-4" : [ "live.dial[4]", "live.dial", 0 ],
			"obj-3::obj-8" : [ "live.grid[5]", "live.grid", 0 ],
			"obj-4" : [ "pushtext4[1]", "pushtext4", 0 ],
			"obj-40::obj-112" : [ "live.text[70]", "live.text", 0 ],
			"obj-40::obj-114" : [ "live.text[71]", "live.text", 0 ],
			"obj-40::obj-116" : [ "live.text[78]", "live.text", 0 ],
			"obj-40::obj-118" : [ "live.text[79]", "live.text", 0 ],
			"obj-40::obj-132" : [ "live.text[1]", "live.text", 0 ],
			"obj-40::obj-134" : [ "live.text[2]", "live.text", 0 ],
			"obj-40::obj-136" : [ "live.text[3]", "live.text", 0 ],
			"obj-40::obj-138" : [ "live.text[4]", "live.text", 0 ],
			"obj-40::obj-141" : [ "live.text[5]", "live.text", 0 ],
			"obj-40::obj-143" : [ "live.text[6]", "live.text", 0 ],
			"obj-40::obj-145" : [ "live.text[7]", "live.text", 0 ],
			"obj-40::obj-147" : [ "live.text[8]", "live.text", 0 ],
			"obj-40::obj-149" : [ "live.text[9]", "live.text", 0 ],
			"obj-40::obj-151" : [ "live.text[10]", "live.text", 0 ],
			"obj-40::obj-153" : [ "live.text[11]", "live.text", 0 ],
			"obj-40::obj-155" : [ "live.text[12]", "live.text", 0 ],
			"obj-40::obj-157" : [ "live.text[13]", "live.text", 0 ],
			"obj-40::obj-159" : [ "live.text[14]", "live.text", 0 ],
			"obj-40::obj-161" : [ "live.text[15]", "live.text", 0 ],
			"obj-40::obj-163" : [ "live.text[16]", "live.text", 0 ],
			"obj-40::obj-165" : [ "live.text[17]", "live.text", 0 ],
			"obj-40::obj-167" : [ "live.text[18]", "live.text", 0 ],
			"obj-40::obj-169" : [ "live.text[19]", "live.text", 0 ],
			"obj-40::obj-171" : [ "live.text[20]", "live.text", 0 ],
			"obj-40::obj-189" : [ "live.text[24]", "live.text", 0 ],
			"obj-40::obj-191" : [ "live.text[28]", "live.text", 0 ],
			"obj-40::obj-193" : [ "live.text[29]", "live.text", 0 ],
			"obj-40::obj-195" : [ "live.text[30]", "live.text", 0 ],
			"obj-40::obj-197" : [ "live.text[31]", "live.text", 0 ],
			"obj-40::obj-199" : [ "live.text[32]", "live.text", 0 ],
			"obj-40::obj-201" : [ "live.text[33]", "live.text", 0 ],
			"obj-40::obj-203" : [ "live.text[34]", "live.text", 0 ],
			"obj-47" : [ "2[3]", "2", 0 ],
			"obj-48" : [ "2[4]", "2", 0 ],
			"obj-49" : [ "2[5]", "2", 0 ],
			"obj-50" : [ "2[6]", "2", 0 ],
			"obj-51" : [ "2[7]", "2", 0 ],
			"obj-6" : [ "live.grid", "live.grid", 0 ],
			"obj-63" : [ "2[8]", "2", 0 ],
			"obj-64" : [ "2[9]", "2", 0 ],
			"obj-65" : [ "2[10]", "2", 0 ],
			"obj-66" : [ "2[11]", "2", 0 ],
			"obj-78" : [ "2[12]", "2", 0 ],
			"obj-79" : [ "2[13]", "2", 0 ],
			"obj-7::obj-104" : [ "-[12]", "1", 0 ],
			"obj-7::obj-105" : [ "-[13]", "1", 0 ],
			"obj-7::obj-106" : [ "-[14]", "1", 0 ],
			"obj-7::obj-107" : [ "-[15]", "1", 0 ],
			"obj-7::obj-142" : [ "mc.live.gain~[1]", "mc.live.gain~[1]", 0 ],
			"obj-7::obj-15" : [ "button[7]", "button[7]", 0 ],
			"obj-7::obj-159" : [ "live.button[10]", "live.button[1]", 0 ],
			"obj-7::obj-162" : [ "live.button[4]", "live.button[1]", 0 ],
			"obj-7::obj-165" : [ "live.button[5]", "live.button[1]", 0 ],
			"obj-7::obj-168" : [ "button", "button", 0 ],
			"obj-7::obj-172" : [ "live.button[6]", "live.button[1]", 0 ],
			"obj-7::obj-175" : [ "live.button[7]", "live.button[1]", 0 ],
			"obj-7::obj-178" : [ "live.button[8]", "live.button[1]", 0 ],
			"obj-7::obj-181" : [ "live.button[9]", "live.button[1]", 0 ],
			"obj-7::obj-224" : [ "-", "1", 0 ],
			"obj-7::obj-225" : [ "-[1]", "1", 0 ],
			"obj-7::obj-226" : [ "-[2]", "1", 0 ],
			"obj-7::obj-227" : [ "-[3]", "1", 0 ],
			"obj-7::obj-228" : [ "-[4]", "1", 0 ],
			"obj-7::obj-229" : [ "-[5]", "1", 0 ],
			"obj-7::obj-230" : [ "-[6]", "1", 0 ],
			"obj-7::obj-231" : [ "-[7]", "1", 0 ],
			"obj-7::obj-62" : [ "live.menu", "live.menu", 0 ],
			"obj-7::obj-70" : [ "live.button[2]", "live.button[1]", 0 ],
			"obj-7::obj-79" : [ "live.numbox", "live.numbox", 0 ],
			"obj-7::obj-89" : [ "flonum[1]", "flonum", 0 ],
			"obj-7::obj-9" : [ "multislider", "multislider", 0 ],
			"obj-7::obj-90" : [ "number[6]", "number[2]", 0 ],
			"obj-7::obj-91" : [ "number[7]", "number[2]", 0 ],
			"obj-7::obj-92" : [ "number[8]", "number[2]", 0 ],
			"obj-7::obj-96" : [ "-[8]", "1", 0 ],
			"obj-7::obj-97" : [ "-[9]", "1", 0 ],
			"obj-7::obj-98" : [ "-[10]", "1", 0 ],
			"obj-7::obj-99" : [ "-[11]", "1", 0 ],
			"obj-80" : [ "2[14]", "2", 0 ],
			"obj-81" : [ "2[15]", "2", 0 ],
			"obj-84" : [ "2[16]", "2", 0 ],
			"obj-85" : [ "2[17]", "2", 0 ],
			"obj-86" : [ "2[18]", "2", 0 ],
			"obj-88" : [ "2[19]", "2", 0 ],
			"obj-89" : [ "2[20]", "2", 0 ],
			"obj-8::obj-4" : [ "live.dial[8]", "live.dial", 0 ],
			"obj-8::obj-8" : [ "live.grid[6]", "live.grid", 0 ],
			"obj-90" : [ "2[21]", "2", 0 ],
			"obj-91" : [ "2[22]", "2", 0 ],
			"obj-93" : [ "2[23]", "2", 0 ],
			"obj-94" : [ "2[24]", "2", 0 ],
			"obj-95" : [ "2[25]", "2", 0 ],
			"obj-96" : [ "2[26]", "2", 0 ],
			"obj-98" : [ "2[27]", "2", 0 ],
			"obj-99" : [ "2[28]", "2", 0 ],
			"obj-9::obj-20" : [ "live.menu[1]", "live.menu", 0 ],
			"obj-9::obj-4" : [ "live.dial[1]", "live.dial", 0 ],
			"parameterbanks" : 			{
				"0" : 				{
					"index" : 0,
					"name" : "",
					"parameters" : [ "-", "-", "-", "-", "-", "-", "-", "-" ]
				}

			}
,
			"parameter_overrides" : 			{
				"obj-10::obj-20" : 				{
					"parameter_longname" : "live.menu[3]"
				}
,
				"obj-10::obj-4" : 				{
					"parameter_longname" : "live.dial[3]"
				}
,
				"obj-11::obj-104" : 				{
					"parameter_longname" : "-[21]"
				}
,
				"obj-11::obj-105" : 				{
					"parameter_longname" : "-[25]"
				}
,
				"obj-11::obj-106" : 				{
					"parameter_longname" : "-[22]"
				}
,
				"obj-11::obj-107" : 				{
					"parameter_longname" : "-[30]"
				}
,
				"obj-11::obj-142" : 				{
					"parameter_longname" : "mc.live.gain~[3]"
				}
,
				"obj-11::obj-159" : 				{
					"parameter_longname" : "live.button[17]"
				}
,
				"obj-11::obj-162" : 				{
					"parameter_longname" : "live.button[11]"
				}
,
				"obj-11::obj-165" : 				{
					"parameter_longname" : "live.button[12]"
				}
,
				"obj-11::obj-172" : 				{
					"parameter_longname" : "live.button[13]"
				}
,
				"obj-11::obj-175" : 				{
					"parameter_longname" : "live.button[14]"
				}
,
				"obj-11::obj-178" : 				{
					"parameter_longname" : "live.button[15]"
				}
,
				"obj-11::obj-181" : 				{
					"parameter_longname" : "live.button[16]"
				}
,
				"obj-11::obj-224" : 				{
					"parameter_longname" : "-[17]"
				}
,
				"obj-11::obj-225" : 				{
					"parameter_longname" : "-[16]"
				}
,
				"obj-11::obj-226" : 				{
					"parameter_longname" : "-[24]"
				}
,
				"obj-11::obj-227" : 				{
					"parameter_longname" : "-[29]"
				}
,
				"obj-11::obj-228" : 				{
					"parameter_longname" : "-[26]"
				}
,
				"obj-11::obj-229" : 				{
					"parameter_longname" : "-[28]"
				}
,
				"obj-11::obj-230" : 				{
					"parameter_longname" : "-[31]"
				}
,
				"obj-11::obj-231" : 				{
					"parameter_longname" : "-[23]"
				}
,
				"obj-11::obj-62" : 				{
					"parameter_longname" : "live.menu[2]"
				}
,
				"obj-11::obj-70" : 				{
					"parameter_longname" : "live.button[3]"
				}
,
				"obj-11::obj-79" : 				{
					"parameter_longname" : "live.numbox[1]"
				}
,
				"obj-11::obj-96" : 				{
					"parameter_longname" : "-[18]"
				}
,
				"obj-11::obj-97" : 				{
					"parameter_longname" : "-[20]"
				}
,
				"obj-11::obj-98" : 				{
					"parameter_longname" : "-[19]"
				}
,
				"obj-11::obj-99" : 				{
					"parameter_longname" : "-[27]"
				}
,
				"obj-12::obj-159" : 				{
					"parameter_longname" : "live.button[53]"
				}
,
				"obj-12::obj-162" : 				{
					"parameter_longname" : "live.button[49]"
				}
,
				"obj-12::obj-165" : 				{
					"parameter_longname" : "live.button[46]"
				}
,
				"obj-12::obj-172" : 				{
					"parameter_longname" : "live.button[50]"
				}
,
				"obj-12::obj-175" : 				{
					"parameter_longname" : "live.button[51]"
				}
,
				"obj-12::obj-178" : 				{
					"parameter_longname" : "live.button[47]"
				}
,
				"obj-12::obj-181" : 				{
					"parameter_longname" : "live.button[52]"
				}
,
				"obj-12::obj-70" : 				{
					"parameter_longname" : "live.button[48]"
				}
,
				"obj-13::obj-20" : 				{
					"parameter_longname" : "live.menu[5]"
				}
,
				"obj-13::obj-4" : 				{
					"parameter_longname" : "live.dial[7]"
				}
,
				"obj-14::obj-104" : 				{
					"parameter_longname" : "-[32]"
				}
,
				"obj-14::obj-105" : 				{
					"parameter_longname" : "-[36]"
				}
,
				"obj-14::obj-106" : 				{
					"parameter_longname" : "-[37]"
				}
,
				"obj-14::obj-107" : 				{
					"parameter_longname" : "-[40]"
				}
,
				"obj-14::obj-142" : 				{
					"parameter_longname" : "mc.live.gain~[4]"
				}
,
				"obj-14::obj-159" : 				{
					"parameter_longname" : "live.button[43]"
				}
,
				"obj-14::obj-162" : 				{
					"parameter_longname" : "live.button[42]"
				}
,
				"obj-14::obj-165" : 				{
					"parameter_longname" : "live.button[41]"
				}
,
				"obj-14::obj-172" : 				{
					"parameter_longname" : "live.button[39]"
				}
,
				"obj-14::obj-175" : 				{
					"parameter_longname" : "live.button[44]"
				}
,
				"obj-14::obj-178" : 				{
					"parameter_longname" : "live.button[40]"
				}
,
				"obj-14::obj-181" : 				{
					"parameter_longname" : "live.button[38]"
				}
,
				"obj-14::obj-224" : 				{
					"parameter_longname" : "-[47]"
				}
,
				"obj-14::obj-225" : 				{
					"parameter_longname" : "-[45]"
				}
,
				"obj-14::obj-226" : 				{
					"parameter_longname" : "-[44]"
				}
,
				"obj-14::obj-227" : 				{
					"parameter_longname" : "-[43]"
				}
,
				"obj-14::obj-228" : 				{
					"parameter_longname" : "-[42]"
				}
,
				"obj-14::obj-229" : 				{
					"parameter_longname" : "-[46]"
				}
,
				"obj-14::obj-230" : 				{
					"parameter_longname" : "-[33]"
				}
,
				"obj-14::obj-231" : 				{
					"parameter_longname" : "-[38]"
				}
,
				"obj-14::obj-62" : 				{
					"parameter_longname" : "live.menu[4]"
				}
,
				"obj-14::obj-70" : 				{
					"parameter_longname" : "live.button[45]"
				}
,
				"obj-14::obj-79" : 				{
					"parameter_longname" : "live.numbox[2]"
				}
,
				"obj-14::obj-96" : 				{
					"parameter_longname" : "-[34]"
				}
,
				"obj-14::obj-97" : 				{
					"parameter_longname" : "-[39]"
				}
,
				"obj-14::obj-98" : 				{
					"parameter_longname" : "-[41]"
				}
,
				"obj-14::obj-99" : 				{
					"parameter_longname" : "-[35]"
				}
,
				"obj-15::obj-159" : 				{
					"parameter_longname" : "live.button[22]"
				}
,
				"obj-15::obj-162" : 				{
					"parameter_longname" : "live.button[24]"
				}
,
				"obj-15::obj-165" : 				{
					"parameter_longname" : "live.button[27]"
				}
,
				"obj-15::obj-172" : 				{
					"parameter_longname" : "live.button[29]"
				}
,
				"obj-15::obj-175" : 				{
					"parameter_longname" : "live.button[23]"
				}
,
				"obj-15::obj-178" : 				{
					"parameter_longname" : "live.button[26]"
				}
,
				"obj-15::obj-181" : 				{
					"parameter_longname" : "live.button[25]"
				}
,
				"obj-15::obj-70" : 				{
					"parameter_longname" : "live.button[28]"
				}
,
				"obj-16::obj-4" : 				{
					"parameter_longname" : "live.dial[11]"
				}
,
				"obj-17::obj-159" : 				{
					"parameter_longname" : "live.button[64]"
				}
,
				"obj-17::obj-162" : 				{
					"parameter_longname" : "live.button[62]"
				}
,
				"obj-17::obj-165" : 				{
					"parameter_longname" : "live.button[68]"
				}
,
				"obj-17::obj-172" : 				{
					"parameter_longname" : "live.button[66]"
				}
,
				"obj-17::obj-175" : 				{
					"parameter_longname" : "live.button[67]"
				}
,
				"obj-17::obj-178" : 				{
					"parameter_longname" : "live.button[69]"
				}
,
				"obj-17::obj-181" : 				{
					"parameter_longname" : "live.button[63]"
				}
,
				"obj-17::obj-70" : 				{
					"parameter_longname" : "live.button[65]"
				}
,
				"obj-18::obj-20" : 				{
					"parameter_longname" : "live.menu[7]"
				}
,
				"obj-18::obj-4" : 				{
					"parameter_longname" : "live.dial[10]"
				}
,
				"obj-19::obj-104" : 				{
					"parameter_longname" : "-[48]"
				}
,
				"obj-19::obj-105" : 				{
					"parameter_longname" : "-[50]"
				}
,
				"obj-19::obj-106" : 				{
					"parameter_longname" : "-[54]"
				}
,
				"obj-19::obj-107" : 				{
					"parameter_longname" : "-[58]"
				}
,
				"obj-19::obj-142" : 				{
					"parameter_longname" : "mc.live.gain~[5]"
				}
,
				"obj-19::obj-159" : 				{
					"parameter_longname" : "live.button[61]"
				}
,
				"obj-19::obj-162" : 				{
					"parameter_longname" : "live.button[60]"
				}
,
				"obj-19::obj-165" : 				{
					"parameter_longname" : "live.button[54]"
				}
,
				"obj-19::obj-172" : 				{
					"parameter_longname" : "live.button[55]"
				}
,
				"obj-19::obj-175" : 				{
					"parameter_longname" : "live.button[57]"
				}
,
				"obj-19::obj-178" : 				{
					"parameter_longname" : "live.button[58]"
				}
,
				"obj-19::obj-181" : 				{
					"parameter_longname" : "live.button[59]"
				}
,
				"obj-19::obj-224" : 				{
					"parameter_longname" : "-[60]"
				}
,
				"obj-19::obj-225" : 				{
					"parameter_longname" : "-[56]"
				}
,
				"obj-19::obj-226" : 				{
					"parameter_longname" : "-[52]"
				}
,
				"obj-19::obj-227" : 				{
					"parameter_longname" : "-[49]"
				}
,
				"obj-19::obj-228" : 				{
					"parameter_longname" : "-[51]"
				}
,
				"obj-19::obj-229" : 				{
					"parameter_longname" : "-[55]"
				}
,
				"obj-19::obj-230" : 				{
					"parameter_longname" : "-[59]"
				}
,
				"obj-19::obj-231" : 				{
					"parameter_longname" : "-[62]"
				}
,
				"obj-19::obj-62" : 				{
					"parameter_longname" : "live.menu[6]"
				}
,
				"obj-19::obj-70" : 				{
					"parameter_longname" : "live.button[56]"
				}
,
				"obj-19::obj-79" : 				{
					"parameter_longname" : "live.numbox[3]"
				}
,
				"obj-19::obj-96" : 				{
					"parameter_longname" : "-[61]"
				}
,
				"obj-19::obj-97" : 				{
					"parameter_longname" : "-[63]"
				}
,
				"obj-19::obj-98" : 				{
					"parameter_longname" : "-[53]"
				}
,
				"obj-19::obj-99" : 				{
					"parameter_longname" : "-[57]"
				}
,
				"obj-1::obj-159" : 				{
					"parameter_longname" : "live.button[35]"
				}
,
				"obj-1::obj-162" : 				{
					"parameter_longname" : "live.button[31]"
				}
,
				"obj-1::obj-165" : 				{
					"parameter_longname" : "live.button[34]"
				}
,
				"obj-1::obj-172" : 				{
					"parameter_longname" : "live.button[32]"
				}
,
				"obj-1::obj-175" : 				{
					"parameter_longname" : "live.button[36]"
				}
,
				"obj-1::obj-178" : 				{
					"parameter_longname" : "live.button[33]"
				}
,
				"obj-1::obj-181" : 				{
					"parameter_longname" : "live.button[30]"
				}
,
				"obj-1::obj-70" : 				{
					"parameter_longname" : "live.button[37]"
				}
,
				"obj-2::obj-4" : 				{
					"parameter_longname" : "live.dial[5]"
				}
,
				"obj-3::obj-4" : 				{
					"parameter_longname" : "live.dial[4]"
				}
,
				"obj-7::obj-159" : 				{
					"parameter_longname" : "live.button[10]"
				}
,
				"obj-8::obj-4" : 				{
					"parameter_longname" : "live.dial[8]"
				}
,
				"obj-9::obj-4" : 				{
					"parameter_longname" : "live.dial[1]"
				}

			}
,
			"inherited_shortname" : 1
		}
,
		"parameter_map" : 		{
			"key" : 			{
				"live.button[28]" : 				{
					"srcname" : "0.modifiers.18.code.key",
					"min" : 0.0,
					"max" : 1.0,
					"flags" : 1
				}
,
				"live.button[22]" : 				{
					"srcname" : "0.modifiers.19.code.key",
					"min" : 0.0,
					"max" : 1.0,
					"flags" : 1
				}
,
				"live.button[24]" : 				{
					"srcname" : "0.modifiers.20.code.key",
					"min" : 0.0,
					"max" : 1.0,
					"flags" : 1
				}
,
				"live.button[27]" : 				{
					"srcname" : "0.modifiers.21.code.key",
					"min" : 0.0,
					"max" : 1.0,
					"flags" : 1
				}
,
				"live.button[25]" : 				{
					"srcname" : "0.modifiers.23.code.key",
					"min" : 0.0,
					"max" : 1.0,
					"flags" : 1
				}
,
				"live.button[26]" : 				{
					"srcname" : "0.modifiers.22.code.key",
					"min" : 0.0,
					"max" : 1.0,
					"flags" : 1
				}
,
				"live.button[23]" : 				{
					"srcname" : "0.modifiers.26.code.key",
					"min" : 0.0,
					"max" : 1.0,
					"flags" : 1
				}
,
				"live.button[29]" : 				{
					"srcname" : "0.modifiers.28.code.key",
					"min" : 0.0,
					"max" : 1.0,
					"flags" : 1
				}

			}
,
			"midi" : 			{
				"live.button[28]" : 				{
					"srcname" : "92.note.0.chan.midi",
					"min" : 0.0,
					"max" : 1.0,
					"flags" : 3,
					"trigger" : 1
				}
,
				"live.button[22]" : 				{
					"srcname" : "93.note.0.chan.midi",
					"min" : 0.0,
					"max" : 1.0,
					"flags" : 3,
					"trigger" : 1
				}
,
				"live.button[24]" : 				{
					"srcname" : "94.note.0.chan.midi",
					"min" : 0.0,
					"max" : 1.0,
					"flags" : 3,
					"trigger" : 1
				}
,
				"live.button[27]" : 				{
					"srcname" : "95.note.0.chan.midi",
					"min" : 0.0,
					"max" : 1.0,
					"flags" : 3,
					"trigger" : 1
				}
,
				"live.button[25]" : 				{
					"srcname" : "96.note.0.chan.midi",
					"min" : 0.0,
					"max" : 1.0,
					"flags" : 3,
					"trigger" : 1
				}
,
				"live.button[26]" : 				{
					"srcname" : "97.note.0.chan.midi",
					"min" : 0.0,
					"max" : 1.0,
					"flags" : 3,
					"trigger" : 1
				}
,
				"live.button[23]" : 				{
					"srcname" : "98.note.0.chan.midi",
					"min" : 0.0,
					"max" : 1.0,
					"flags" : 3,
					"trigger" : 1
				}
,
				"live.button[29]" : 				{
					"srcname" : "99.note.0.chan.midi",
					"min" : 0.0,
					"max" : 1.0,
					"flags" : 3,
					"trigger" : 1
				}
,
				"live.button[65]" : 				{
					"srcname" : "76.note.0.chan.midi",
					"min" : 0.0,
					"max" : 1.0,
					"flags" : 3,
					"trigger" : 1
				}
,
				"live.button[64]" : 				{
					"srcname" : "77.note.0.chan.midi",
					"min" : 0.0,
					"max" : 1.0,
					"flags" : 3,
					"trigger" : 1
				}
,
				"live.button[62]" : 				{
					"srcname" : "78.note.0.chan.midi",
					"min" : 0.0,
					"max" : 1.0,
					"flags" : 3,
					"trigger" : 1
				}
,
				"live.button[68]" : 				{
					"srcname" : "79.note.0.chan.midi",
					"min" : 0.0,
					"max" : 1.0,
					"flags" : 3,
					"trigger" : 1
				}
,
				"live.button[63]" : 				{
					"srcname" : "80.note.0.chan.midi",
					"min" : 0.0,
					"max" : 1.0,
					"flags" : 3,
					"trigger" : 1
				}
,
				"live.button[69]" : 				{
					"srcname" : "81.note.0.chan.midi",
					"min" : 0.0,
					"max" : 1.0,
					"flags" : 3,
					"trigger" : 1
				}
,
				"live.button[67]" : 				{
					"srcname" : "82.note.0.chan.midi",
					"min" : 0.0,
					"max" : 1.0,
					"flags" : 3,
					"trigger" : 1
				}
,
				"live.button[66]" : 				{
					"srcname" : "83.note.0.chan.midi",
					"min" : 0.0,
					"max" : 1.0,
					"flags" : 3,
					"trigger" : 1
				}
,
				"live.button[48]" : 				{
					"srcname" : "60.note.0.chan.midi",
					"min" : 0.0,
					"max" : 1.0,
					"flags" : 3,
					"trigger" : 1
				}
,
				"live.button[53]" : 				{
					"srcname" : "61.note.0.chan.midi",
					"min" : 0.0,
					"max" : 1.0,
					"flags" : 3,
					"trigger" : 1
				}
,
				"live.button[49]" : 				{
					"srcname" : "62.note.0.chan.midi",
					"min" : 0.0,
					"max" : 1.0,
					"flags" : 3,
					"trigger" : 1
				}
,
				"live.button[46]" : 				{
					"srcname" : "63.note.0.chan.midi",
					"min" : 0.0,
					"max" : 1.0,
					"flags" : 3,
					"trigger" : 1
				}
,
				"live.button[52]" : 				{
					"srcname" : "64.note.0.chan.midi",
					"min" : 0.0,
					"max" : 1.0,
					"flags" : 3,
					"trigger" : 1
				}
,
				"live.button[47]" : 				{
					"srcname" : "65.note.0.chan.midi",
					"min" : 0.0,
					"max" : 1.0,
					"flags" : 3,
					"trigger" : 1
				}
,
				"live.button[51]" : 				{
					"srcname" : "66.note.0.chan.midi",
					"min" : 0.0,
					"max" : 1.0,
					"flags" : 3,
					"trigger" : 1
				}
,
				"live.button[50]" : 				{
					"srcname" : "67.note.0.chan.midi",
					"min" : 0.0,
					"max" : 1.0,
					"flags" : 3,
					"trigger" : 1
				}
,
				"live.button[37]" : 				{
					"srcname" : "44.note.0.chan.midi",
					"min" : 0.0,
					"max" : 1.0,
					"flags" : 3,
					"trigger" : 1
				}
,
				"live.button[35]" : 				{
					"srcname" : "45.note.0.chan.midi",
					"min" : 0.0,
					"max" : 1.0,
					"flags" : 3,
					"trigger" : 1
				}
,
				"live.button[31]" : 				{
					"srcname" : "46.note.0.chan.midi",
					"min" : 0.0,
					"max" : 1.0,
					"flags" : 3,
					"trigger" : 1
				}
,
				"live.button[34]" : 				{
					"srcname" : "47.note.0.chan.midi",
					"min" : 0.0,
					"max" : 1.0,
					"flags" : 3,
					"trigger" : 1
				}
,
				"live.button[30]" : 				{
					"srcname" : "48.note.0.chan.midi",
					"min" : 0.0,
					"max" : 1.0,
					"flags" : 3,
					"trigger" : 1
				}
,
				"live.button[33]" : 				{
					"srcname" : "49.note.0.chan.midi",
					"min" : 0.0,
					"max" : 1.0,
					"flags" : 3,
					"trigger" : 1
				}
,
				"live.button[36]" : 				{
					"srcname" : "50.note.0.chan.midi",
					"min" : 0.0,
					"max" : 1.0,
					"flags" : 3,
					"trigger" : 1
				}
,
				"live.button[32]" : 				{
					"srcname" : "51.note.0.chan.midi",
					"min" : 0.0,
					"max" : 1.0,
					"flags" : 3,
					"trigger" : 1
				}
,
				"button[9]" : 				{
					"srcname" : "20.ctrl.0.chan.midi",
					"min" : 0.0,
					"max" : 1.0,
					"flags" : 2
				}
,
				"button[10]" : 				{
					"srcname" : "21.ctrl.0.chan.midi",
					"min" : 0.0,
					"max" : 1.0,
					"flags" : 2
				}
,
				"button[11]" : 				{
					"srcname" : "22.ctrl.0.chan.midi",
					"min" : 0.0,
					"max" : 1.0,
					"flags" : 2
				}
,
				"button[12]" : 				{
					"srcname" : "23.ctrl.0.chan.midi",
					"min" : 0.0,
					"max" : 1.0,
					"flags" : 2
				}

			}

		}
,
		"dependency_cache" : [ 			{
				"name" : "MASTER[1].maxsnap",
				"bootpath" : "~/Documents/Max 9/Snapshots",
				"patcherrelativepath" : "../../../../../../../../Documents/Max 9/Snapshots",
				"type" : "mx@s",
				"implicit" : 1
			}
, 			{
				"name" : "MASTER[1]_20260504.maxsnap",
				"bootpath" : "~/Documents/Max 9/Snapshots",
				"patcherrelativepath" : "../../../../../../../../Documents/Max 9/Snapshots",
				"type" : "mx@s",
				"implicit" : 1
			}
, 			{
				"name" : "MCHelper.maxpat",
				"bootpath" : "~/Library/CloudStorage/GoogleDrive-austin.tecks@gmail.com/My Drive/MDM/schizophonic_remediation/MaxPatches/Tutorials",
				"patcherrelativepath" : ".",
				"type" : "JSON",
				"implicit" : 1
			}
, 			{
				"name" : "MCLyra.maxpat",
				"bootpath" : "~/Library/CloudStorage/GoogleDrive-austin.tecks@gmail.com/My Drive/MDM/schizophonic_remediation/MaxPatches/Tutorials",
				"patcherrelativepath" : ".",
				"type" : "JSON",
				"implicit" : 1
			}
, 			{
				"name" : "MCModule.maxpat",
				"bootpath" : "~/Library/CloudStorage/GoogleDrive-austin.tecks@gmail.com/My Drive/MDM/schizophonic_remediation/MaxPatches/Tutorials",
				"patcherrelativepath" : ".",
				"type" : "JSON",
				"implicit" : 1
			}
, 			{
				"name" : "MCSketch.maxsnap",
				"bootpath" : "~/Documents/Max 9/Snapshots",
				"patcherrelativepath" : "../../../../../../../../Documents/Max 9/Snapshots",
				"type" : "mx@s",
				"implicit" : 1
			}
, 			{
				"name" : "May3[1].maxsnap",
				"bootpath" : "~/Documents/Max 9/Snapshots",
				"patcherrelativepath" : "../../../../../../../../Documents/Max 9/Snapshots",
				"type" : "mx@s",
				"implicit" : 1
			}
, 			{
				"name" : "gridmodule.maxpat",
				"bootpath" : "~/Library/CloudStorage/GoogleDrive-austin.tecks@gmail.com/My Drive/MDM/schizophonic_remediation/MaxPatches/Tutorials",
				"patcherrelativepath" : ".",
				"type" : "JSON",
				"implicit" : 1
			}
, 			{
				"name" : "helpargs.js",
				"bootpath" : "C74:/help/resources",
				"type" : "TEXT",
				"implicit" : 1
			}
, 			{
				"name" : "jam1[1].maxsnap",
				"bootpath" : "~/Documents/Max 9/Snapshots",
				"patcherrelativepath" : "../../../../../../../../Documents/Max 9/Snapshots",
				"type" : "mx@s",
				"implicit" : 1
			}
, 			{
				"name" : "jam1[1]_20260430.maxsnap",
				"bootpath" : "~/Documents/Max 9/Snapshots",
				"patcherrelativepath" : "../../../../../../../../Documents/Max 9/Snapshots",
				"type" : "mx@s",
				"implicit" : 1
			}
, 			{
				"name" : "jam1[1]_20260502.maxsnap",
				"bootpath" : "~/Documents/Max 9/Snapshots",
				"patcherrelativepath" : "../../../../../../../../Documents/Max 9/Snapshots",
				"type" : "mx@s",
				"implicit" : 1
			}
, 			{
				"name" : "jam1[1]_20260503.maxsnap",
				"bootpath" : "~/Documents/Max 9/Snapshots",
				"patcherrelativepath" : "../../../../../../../../Documents/Max 9/Snapshots",
				"type" : "mx@s",
				"implicit" : 1
			}
, 			{
				"name" : "jam1[1]_20260507.maxsnap",
				"bootpath" : "~/Documents/Max 9/Snapshots",
				"patcherrelativepath" : "../../../../../../../../Documents/Max 9/Snapshots",
				"type" : "mx@s",
				"implicit" : 1
			}
, 			{
				"name" : "jam1[1]_20260509.maxsnap",
				"bootpath" : "~/Documents/Max 9/Snapshots",
				"patcherrelativepath" : "../../../../../../../../Documents/Max 9/Snapshots",
				"type" : "mx@s",
				"implicit" : 1
			}
, 			{
				"name" : "jam1[1]_20260510.maxsnap",
				"bootpath" : "~/Documents/Max 9/Snapshots",
				"patcherrelativepath" : "../../../../../../../../Documents/Max 9/Snapshots",
				"type" : "mx@s",
				"implicit" : 1
			}
, 			{
				"name" : "jam1[1]_20260521.maxsnap",
				"bootpath" : "~/Documents/Max 9/Snapshots",
				"patcherrelativepath" : "../../../../../../../../Documents/Max 9/Snapshots",
				"type" : "mx@s",
				"implicit" : 1
			}
, 			{
				"name" : "jam4[1].maxsnap",
				"bootpath" : "~/Documents/Max 9/Snapshots",
				"patcherrelativepath" : "../../../../../../../../Documents/Max 9/Snapshots",
				"type" : "mx@s",
				"implicit" : 1
			}
, 			{
				"name" : "jk.push.maxpat",
				"bootpath" : "~/Documents/Max 9/Packages/jk.push 2/patchers",
				"patcherrelativepath" : "../../../../../../../../Documents/Max 9/Packages/jk.push 2/patchers",
				"type" : "JSON",
				"implicit" : 1
			}
, 			{
				"name" : "jk.pushcore.maxpat",
				"bootpath" : "~/Documents/Max 9/Packages/jk.push 2/patchers",
				"patcherrelativepath" : "../../../../../../../../Documents/Max 9/Packages/jk.push 2/patchers",
				"type" : "JSON",
				"implicit" : 1
			}
, 			{
				"name" : "jk.pushrotaryAlt.maxpat",
				"bootpath" : "~/Documents/Max 9/Packages/jk.push 2/patchers",
				"patcherrelativepath" : "../../../../../../../../Documents/Max 9/Packages/jk.push 2/patchers",
				"type" : "JSON",
				"implicit" : 1
			}
, 			{
				"name" : "livebuttoninfo.maxpat",
				"bootpath" : "~/Documents/Max 9/Packages/jk.push 2/patchers",
				"patcherrelativepath" : "../../../../../../../../Documents/Max 9/Packages/jk.push 2/patchers",
				"type" : "JSON",
				"implicit" : 1
			}
, 			{
				"name" : "livedialinfo.maxpat",
				"bootpath" : "~/Documents/Max 9/Packages/jk.push 2/patchers",
				"patcherrelativepath" : "../../../../../../../../Documents/Max 9/Packages/jk.push 2/patchers",
				"type" : "JSON",
				"implicit" : 1
			}
, 			{
				"name" : "livemenuinfo.maxpat",
				"bootpath" : "~/Documents/Max 9/Packages/jk.push 2/patchers",
				"patcherrelativepath" : "../../../../../../../../Documents/Max 9/Packages/jk.push 2/patchers",
				"type" : "JSON",
				"implicit" : 1
			}
, 			{
				"name" : "livetextinfo.maxpat",
				"bootpath" : "~/Documents/Max 9/Packages/jk.push 2/patchers",
				"patcherrelativepath" : "../../../../../../../../Documents/Max 9/Packages/jk.push 2/patchers",
				"type" : "JSON",
				"implicit" : 1
			}
, 			{
				"name" : "may21[1].maxsnap",
				"bootpath" : "~/Documents/Max 9/Snapshots",
				"patcherrelativepath" : "../../../../../../../../Documents/Max 9/Snapshots",
				"type" : "mx@s",
				"implicit" : 1
			}
 ],
		"autosave" : 0,
		"snapshot" : 		{
			"filetype" : "C74Snapshot",
			"version" : 2,
			"minorversion" : 0,
			"name" : "snapshotlist",
			"origin" : "jpatcher",
			"type" : "list",
			"subtype" : "Undefined",
			"embed" : 1,
			"snapshot" : 			{
				"valuedictionary" : 				{
					"parameter_values" : 					{
						"-" : 1.0,
						"-[10]" : 0.00032,
						"-[11]" : 0.00184,
						"-[12]" : 0.00484,
						"-[13]" : 0.00008,
						"-[14]" : -0.00152,
						"-[15]" : -0.01256,
						"-[16]" : 4.0,
						"-[17]" : 3.0,
						"-[18]" : 0.00556,
						"-[19]" : 0.00216,
						"-[1]" : 3.0,
						"-[20]" : 0.00412,
						"-[21]" : 0.0118,
						"-[22]" : 0.00056,
						"-[23]" : 5.0,
						"-[24]" : 6.0,
						"-[25]" : 0.00196,
						"-[26]" : 10.0,
						"-[27]" : 0.00292,
						"-[28]" : 9.0,
						"-[29]" : 15.0,
						"-[2]" : 5.0,
						"-[30]" : 0.0,
						"-[31]" : 5.0,
						"-[32]" : 0.00536,
						"-[33]" : 6.0,
						"-[34]" : 0.00236,
						"-[35]" : -0.00112,
						"-[36]" : -0.0098,
						"-[37]" : -0.00804,
						"-[38]" : 7.0,
						"-[39]" : -0.00228,
						"-[3]" : 14.0,
						"-[40]" : -0.00748,
						"-[41]" : -0.0084,
						"-[42]" : 10.0,
						"-[43]" : 11.0,
						"-[44]" : 3.0,
						"-[45]" : 5.0,
						"-[46]" : 8.0,
						"-[47]" : 1.0,
						"-[48]" : -0.00412,
						"-[49]" : 14.0,
						"-[4]" : 15.0,
						"-[50]" : 0.00312,
						"-[51]" : 9.0,
						"-[52]" : 3.0,
						"-[53]" : -0.00064,
						"-[54]" : 0.00444,
						"-[55]" : 8.0,
						"-[56]" : 5.0,
						"-[57]" : 0.0044,
						"-[58]" : 0.00436,
						"-[59]" : 6.0,
						"-[5]" : 11.0,
						"-[60]" : 1.0,
						"-[61]" : 0.01132,
						"-[62]" : 5.0,
						"-[63]" : 0.01064,
						"-[6]" : 9.0,
						"-[7]" : 7.0,
						"-[8]" : 0.00452,
						"-[9]" : 0.00512,
						"2" : 186.0,
						"2[10]" : 578.0,
						"2[11]" : 502.0,
						"2[12]" : 573.0,
						"2[13]" : 472.0,
						"2[14]" : 610.0,
						"2[15]" : 546.0,
						"2[16]" : 634.0,
						"2[17]" : 397.0,
						"2[18]" : 621.0,
						"2[19]" : 639.0,
						"2[1]" : 609.0,
						"2[20]" : 559.0,
						"2[21]" : 783.0,
						"2[22]" : 613.0,
						"2[23]" : 603.0,
						"2[24]" : 443.0,
						"2[25]" : 766.0,
						"2[26]" : 628.0,
						"2[27]" : 554.0,
						"2[28]" : 290.0,
						"2[29]" : 484.0,
						"2[2]" : 313.0,
						"2[30]" : 795.0,
						"2[31]" : 508.0,
						"2[3]" : 500.0,
						"2[4]" : 514.0,
						"2[5]" : 299.0,
						"2[6]" : 611.0,
						"2[7]" : 462.0,
						"2[8]" : 549.0,
						"2[9]" : 255.0,
						"button" : 0.0,
						"button[1]" : 0.0,
						"button[2]" : 0.0,
						"button[3]" : 0.0,
						"button[4]" : 0.0,
						"button[5]" : 0.0,
						"button[7]" : 0.0,
						"button[8]" : 0.0,
						"live.button[10]" : 0.0,
						"live.button[11]" : 0.0,
						"live.button[12]" : 0.0,
						"live.button[13]" : 0.0,
						"live.button[14]" : 0.0,
						"live.button[15]" : 0.0,
						"live.button[16]" : 0.0,
						"live.button[17]" : 0.0,
						"live.button[22]" : 0.0,
						"live.button[23]" : 0.0,
						"live.button[24]" : 0.0,
						"live.button[25]" : 0.0,
						"live.button[26]" : 0.0,
						"live.button[27]" : 0.0,
						"live.button[28]" : 0.0,
						"live.button[29]" : 0.0,
						"live.button[2]" : 0.0,
						"live.button[30]" : 0.0,
						"live.button[31]" : 0.0,
						"live.button[32]" : 0.0,
						"live.button[33]" : 0.0,
						"live.button[34]" : 0.0,
						"live.button[35]" : 0.0,
						"live.button[36]" : 0.0,
						"live.button[37]" : 0.0,
						"live.button[38]" : 0.0,
						"live.button[39]" : 0.0,
						"live.button[3]" : 0.0,
						"live.button[40]" : 0.0,
						"live.button[41]" : 0.0,
						"live.button[42]" : 0.0,
						"live.button[43]" : 0.0,
						"live.button[44]" : 0.0,
						"live.button[45]" : 0.0,
						"live.button[46]" : 0.0,
						"live.button[47]" : 0.0,
						"live.button[48]" : 0.0,
						"live.button[49]" : 0.0,
						"live.button[4]" : 0.0,
						"live.button[50]" : 0.0,
						"live.button[51]" : 0.0,
						"live.button[52]" : 0.0,
						"live.button[53]" : 0.0,
						"live.button[54]" : 0.0,
						"live.button[55]" : 0.0,
						"live.button[56]" : 0.0,
						"live.button[57]" : 0.0,
						"live.button[58]" : 0.0,
						"live.button[59]" : 0.0,
						"live.button[5]" : 0.0,
						"live.button[60]" : 0.0,
						"live.button[61]" : 0.0,
						"live.button[62]" : 0.0,
						"live.button[63]" : 0.0,
						"live.button[64]" : 0.0,
						"live.button[65]" : 0.0,
						"live.button[66]" : 0.0,
						"live.button[67]" : 0.0,
						"live.button[68]" : 0.0,
						"live.button[69]" : 0.0,
						"live.button[6]" : 0.0,
						"live.button[7]" : 0.0,
						"live.button[8]" : 0.0,
						"live.button[9]" : 0.0,
						"live.dial[10]" : 108.682055549781055,
						"live.dial[11]" : 6173.0,
						"live.dial[1]" : 70.14173228346354,
						"live.dial[3]" : 257.826207108760343,
						"live.dial[4]" : 6519.000000000000909,
						"live.dial[5]" : 6454.000000000001819,
						"live.dial[7]" : 135.233687156586029,
						"live.dial[8]" : 6917.0,
						"live.gain~" : -70.0,
						"live.menu" : 1.0,
						"live.menu[1]" : 0.0,
						"live.menu[2]" : 1.0,
						"live.menu[3]" : 3.0,
						"live.menu[4]" : 1.0,
						"live.menu[5]" : 2.0,
						"live.menu[6]" : 1.0,
						"live.menu[7]" : 1.0,
						"live.numbox" : 0.0,
						"live.numbox[1]" : 0.0,
						"live.numbox[2]" : 0.0,
						"live.numbox[3]" : 0.0,
						"live.text[10]" : 0.0,
						"live.text[11]" : 0.0,
						"live.text[12]" : 0.0,
						"live.text[13]" : 0.0,
						"live.text[14]" : 0.0,
						"live.text[15]" : 0.0,
						"live.text[16]" : 0.0,
						"live.text[17]" : 0.0,
						"live.text[18]" : 0.0,
						"live.text[19]" : 0.0,
						"live.text[1]" : 0.0,
						"live.text[20]" : 0.0,
						"live.text[24]" : 0.0,
						"live.text[28]" : 0.0,
						"live.text[29]" : 0.0,
						"live.text[2]" : 0.0,
						"live.text[30]" : 0.0,
						"live.text[31]" : 0.0,
						"live.text[32]" : 0.0,
						"live.text[33]" : 0.0,
						"live.text[34]" : 0.0,
						"live.text[3]" : 0.0,
						"live.text[4]" : 0.0,
						"live.text[5]" : 0.0,
						"live.text[6]" : 0.0,
						"live.text[70]" : 0.0,
						"live.text[71]" : 0.0,
						"live.text[78]" : 0.0,
						"live.text[79]" : 0.0,
						"live.text[7]" : 0.0,
						"live.text[8]" : 0.0,
						"live.text[9]" : 0.0,
						"mc.live.gain~[1]" : 0.0,
						"mc.live.gain~[3]" : 0.0,
						"mc.live.gain~[4]" : 0.0,
						"mc.live.gain~[5]" : 0.0,
						"pushtext16[1]" : 0.0,
						"pushtext18[1]" : 0.0,
						"pushtext20[1]" : 0.0,
						"pushtext22[1]" : 1.0,
						"pushtext2[1]" : 0.0,
						"pushtext3[1]" : 0.0,
						"pushtext4[1]" : 0.0,
						"pushtext5[1]" : 0.0,
						"blob" : 						{
							"flonum[1]" : [ 0.5 ],
							"flonum[2]" : [ 0.5 ],
							"flonum[3]" : [ 1.0 ],
							"flonum[4]" : [ 1.0 ],
							"flonum[5]" : [ 0.5 ],
							"flonum[6]" : [ 1.0 ],
							"flonum[7]" : [ 0.5 ],
							"flonum[8]" : [ 1.0 ],
							"live.grid" : [ 0, 16, 16, 0, 16, 0, 1001, 2002, 3003, 4004, 5005, 6006, 7007, 8008, 9009, 10010, 11011, 12012, 13013, 14014, 15015, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 ],
							"live.grid[1]" : [ 3, 16, 8, 0, 15, 7, 1004, 2006, 3004, 4003, 5002, 6007, 7004, 8007, 9004, 10006, 11001, 13004, 14001, 15005, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 ],
							"live.grid[2]" : [ 3, 16, 8, 0, 5, 2002, 3000, 8002, 12002, 15005, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 ],
							"live.grid[5]" : [ 3, 16, 8, 0, 6, 0, 3001, 7000, 10001, 13005, 14000, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 ],
							"live.grid[6]" : [ 3, 16, 8, 0, 7, 2005, 5006, 6004, 9001, 11004, 12005, 14001, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 ],
							"multislider" : [ 0.600477832363498, 1.0, 1.0, 1.0, 1.0, 1.0, 0.77570683879237, 0.911111071032862 ],
							"multislider[1]" : [ 1.0, 1.0, 1.0, 0.991756252319582, 1.0, 1.0, 1.0, 1.0 ],
							"multislider[2]" : [ 0.727917109766314, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 ],
							"multislider[3]" : [ 0.648267561389554, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 ],
							"number[10]" : [ 100 ],
							"number[11]" : [ 1000 ],
							"number[12]" : [ 1288 ],
							"number[13]" : [ 100 ],
							"number[14]" : [ 303 ],
							"number[15]" : [ 100 ],
							"number[16]" : [ 50000 ],
							"number[17]" : [ 1695 ],
							"number[18]" : [ 600 ],
							"number[19]" : [ 100 ],
							"number[20]" : [ 1000 ],
							"number[21]" : [ 2581 ],
							"number[22]" : [ 100 ],
							"number[23]" : [ 615 ],
							"number[24]" : [ 600 ],
							"number[25]" : [ 100 ],
							"number[26]" : [ 1000 ],
							"number[27]" : [ 602 ],
							"number[28]" : [ 100 ],
							"number[29]" : [ 1052 ],
							"number[6]" : [ 1000 ],
							"number[7]" : [ 100 ],
							"number[8]" : [ 600 ],
							"number[9]" : [ 600 ]
						}

					}

				}

			}
,
			"snapshotlist" : 			{
				"current_snapshot" : 13,
				"entries" : [ 					{
						"filetype" : "C74Snapshot",
						"version" : 2,
						"minorversion" : 0,
						"name" : "jam1",
						"origin" : "MCSketch",
						"type" : "patcher",
						"subtype" : "Undefined",
						"embed" : 0,
						"fileref" : 						{
							"name" : "jam1",
							"filename" : "MCSketch.maxsnap",
							"filepath" : "~/Documents/Max 9/Snapshots",
							"filepos" : -1,
							"snapshotfileid" : "06a081ec7f3c47c51dfc30b1fbc2c071"
						}

					}
, 					{
						"filetype" : "C74Snapshot",
						"version" : 2,
						"minorversion" : 0,
						"name" : "lolo",
						"origin" : "MCSketch",
						"type" : "patcher",
						"subtype" : "Undefined",
						"embed" : 0,
						"snapshot" : 						{
							"valuedictionary" : 							{
								"parameter_values" : 								{
									"-" : 1.0,
									"-[10]" : 0.004094488188976,
									"-[11]" : -0.001984251968504,
									"-[12]" : 0.005511811023622,
									"-[13]" : 0.000976377952756,
									"-[14]" : 0.003779527559055,
									"-[15]" : 0.001889763779528,
									"-[16]" : 3.0,
									"-[17]" : 1.0,
									"-[18]" : -0.002204724409449,
									"-[19]" : 0.0,
									"-[1]" : 2.0,
									"-[20]" : 0.0,
									"-[21]" : -0.000944881889764,
									"-[22]" : 0.000629921259843,
									"-[23]" : 6.0,
									"-[24]" : 4.0,
									"-[25]" : 0.0,
									"-[26]" : 10.0,
									"-[27]" : -0.0,
									"-[28]" : 10.0,
									"-[29]" : 12.0,
									"-[2]" : 5.0,
									"-[30]" : 0.0,
									"-[31]" : 9.0,
									"-[32]" : 0.00755905511811,
									"-[33]" : 9.0,
									"-[34]" : 0.0,
									"-[35]" : 0.001574803149606,
									"-[36]" : -0.003464566929134,
									"-[37]" : 0.0,
									"-[38]" : 7.0,
									"-[39]" : 0.000944881889764,
									"-[3]" : 13.0,
									"-[40]" : 0.0,
									"-[41]" : 0.003149606299213,
									"-[42]" : 12.0,
									"-[43]" : 13.0,
									"-[44]" : 5.0,
									"-[45]" : 3.0,
									"-[46]" : 10.0,
									"-[47]" : 2.0,
									"-[4]" : 12.0,
									"-[5]" : 10.0,
									"-[6]" : 8.0,
									"-[7]" : 6.0,
									"-[8]" : 0.00503937007874,
									"-[9]" : 0.003464566929134,
									"button" : 0.0,
									"button[1]" : 0.0,
									"button[2]" : 0.0,
									"button[3]" : 0.0,
									"button[7]" : 0.0,
									"button[8]" : 0.0,
									"live.button[10]" : 0.0,
									"live.button[11]" : 0.0,
									"live.button[12]" : 0.0,
									"live.button[13]" : 0.0,
									"live.button[14]" : 0.0,
									"live.button[15]" : 0.0,
									"live.button[16]" : 0.0,
									"live.button[17]" : 0.0,
									"live.button[22]" : 0.0,
									"live.button[23]" : 0.0,
									"live.button[24]" : 0.0,
									"live.button[25]" : 0.0,
									"live.button[26]" : 0.0,
									"live.button[27]" : 0.0,
									"live.button[28]" : 0.0,
									"live.button[29]" : 0.0,
									"live.button[2]" : 0.0,
									"live.button[30]" : 0.0,
									"live.button[31]" : 0.0,
									"live.button[32]" : 0.0,
									"live.button[33]" : 0.0,
									"live.button[34]" : 0.0,
									"live.button[35]" : 0.0,
									"live.button[36]" : 0.0,
									"live.button[37]" : 0.0,
									"live.button[38]" : 0.0,
									"live.button[39]" : 0.0,
									"live.button[3]" : 0.0,
									"live.button[40]" : 0.0,
									"live.button[41]" : 0.0,
									"live.button[42]" : 0.0,
									"live.button[43]" : 0.0,
									"live.button[44]" : 0.0,
									"live.button[45]" : 0.0,
									"live.button[46]" : 0.0,
									"live.button[47]" : 0.0,
									"live.button[48]" : 0.0,
									"live.button[49]" : 0.0,
									"live.button[4]" : 0.0,
									"live.button[50]" : 0.0,
									"live.button[51]" : 0.0,
									"live.button[52]" : 0.0,
									"live.button[53]" : 0.0,
									"live.button[5]" : 0.0,
									"live.button[6]" : 0.0,
									"live.button[7]" : 0.0,
									"live.button[8]" : 0.0,
									"live.button[9]" : 0.0,
									"live.dial" : 275.275590551180642,
									"live.dial[1]" : 78.362204724409409,
									"live.dial[2]" : 301.080681280403155,
									"live.dial[3]" : 284.343495703500423,
									"live.dial[4]" : 557.000000000000341,
									"live.dial[5]" : 408.000000000000057,
									"live.dial[6]" : 285.999999999999943,
									"live.dial[7]" : 107.195470434273886,
									"live.dial[8]" : 1000.0,
									"live.gain~" : 0.0,
									"live.menu" : 1.0,
									"live.menu[1]" : 0.0,
									"live.menu[2]" : 1.0,
									"live.menu[3]" : 1.0,
									"live.menu[4]" : 1.0,
									"live.menu[5]" : 2.0,
									"live.numbox" : 0.0,
									"live.numbox[1]" : 0.0,
									"live.numbox[2]" : 0.503937007874016,
									"mc.live.gain~[1]" : 0.0,
									"mc.live.gain~[3]" : 0.0,
									"mc.live.gain~[4]" : 0.0,
									"blob" : 									{
										"flonum[1]" : [ 0.5 ],
										"flonum[2]" : [ 0.5 ],
										"flonum[3]" : [ 1.0 ],
										"flonum[4]" : [ 1.0 ],
										"flonum[5]" : [ 0.5 ],
										"flonum[6]" : [ 1.0 ],
										"live.grid[1]" : [ 3, 16, 8, 0, 21, 7, 1006, 2003, 2007, 3006, 4007, 5003, 5006, 6007, 7006, 8003, 8007, 9006, 10007, 11003, 11006, 12007, 13006, 14003, 14007, 15006, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 ],
										"live.grid[5]" : [ 3, 16, 8, 0, 11, 0, 1000, 3004, 4004, 6003, 7003, 9002, 10002, 12005, 13005, 15001, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 ],
										"live.grid[6]" : [ 3, 16, 8, 0, 0, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 ],
										"multislider" : [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ],
										"multislider[1]" : [ 0.606451612903226, 0.638709677419355, 0.72258064516129, 0.819354838709677, 0.716129032258065, 0.806451612903226, 1.0, 0.748387096774194 ],
										"multislider[2]" : [ 0.720534995294386, 0.405822163243448, 0.846420128114762, 0.877891411319856, 0.870023590518582, 0.822816665710941, 0.807081024108395, 0.807081024108395 ],
										"number[10]" : [ 100 ],
										"number[11]" : [ 1000 ],
										"number[12]" : [ 1000 ],
										"number[13]" : [ 100 ],
										"number[14]" : [ 149 ],
										"number[15]" : [ 100 ],
										"number[16]" : [ 1682 ],
										"number[17]" : [ 219 ],
										"number[18]" : [ 600 ],
										"number[19]" : [ 100 ],
										"number[20]" : [ 1000 ],
										"number[21]" : [ 1000 ],
										"number[22]" : [ 100 ],
										"number[23]" : [ 600 ],
										"number[6]" : [ 1000 ],
										"number[7]" : [ 100 ],
										"number[8]" : [ 600 ],
										"number[9]" : [ 600 ]
									}

								}

							}

						}
,
						"fileref" : 						{
							"name" : "lolo",
							"filename" : "jam1[1].maxsnap",
							"filepath" : "~/Documents/Max 9/Snapshots",
							"filepos" : -1,
							"snapshotfileid" : "2e9d5032befa3ead820e0f14500f68db"
						}

					}
, 					{
						"filetype" : "C74Snapshot",
						"version" : 2,
						"minorversion" : 0,
						"name" : "jam4",
						"origin" : "MCSketch",
						"type" : "patcher",
						"subtype" : "Undefined",
						"embed" : 0,
						"fileref" : 						{
							"name" : "jam4",
							"filename" : "jam1[1]_20260430.maxsnap",
							"filepath" : "~/Documents/Max 9/Snapshots",
							"filepos" : -1,
							"snapshotfileid" : "8e9097a20e9f7aac6499e3c6846944d7"
						}

					}
, 					{
						"filetype" : "C74Snapshot",
						"version" : 2,
						"minorversion" : 0,
						"name" : "jam4[1]",
						"origin" : "MCSketch",
						"type" : "patcher",
						"subtype" : "Undefined",
						"embed" : 0,
						"fileref" : 						{
							"name" : "jam4[1]",
							"filename" : "jam4[1].maxsnap",
							"filepath" : "~/Documents/Max 9/Snapshots",
							"filepos" : -1,
							"snapshotfileid" : "9da244848fc504d90b77c86d8351d3e1"
						}

					}
, 					{
						"filetype" : "C74Snapshot",
						"version" : 2,
						"minorversion" : 0,
						"name" : "May2",
						"origin" : "MCSketch",
						"type" : "patcher",
						"subtype" : "Undefined",
						"embed" : 0,
						"snapshot" : 						{
							"valuedictionary" : 							{
								"parameter_values" : 								{
									"-" : 1.0,
									"-[10]" : -0.004881889763779,
									"-[11]" : 0.003370078740157,
									"-[12]" : 0.018425196850394,
									"-[13]" : -0.000944881889764,
									"-[14]" : -0.002047244094488,
									"-[15]" : -0.007086614173228,
									"-[16]" : 6.0,
									"-[17]" : 3.0,
									"-[18]" : 0.00251968503937,
									"-[19]" : -0.002834645669291,
									"-[1]" : 3.0,
									"-[20]" : -0.002834645669291,
									"-[21]" : -0.00044094488189,
									"-[22]" : 0.0,
									"-[23]" : 12.0,
									"-[24]" : 12.0,
									"-[25]" : -0.0,
									"-[26]" : 15.0,
									"-[27]" : 0.002204724409449,
									"-[28]" : 10.0,
									"-[29]" : 15.0,
									"-[2]" : 5.0,
									"-[30]" : -0.000629921259843,
									"-[31]" : 5.0,
									"-[32]" : 0.006141732283465,
									"-[33]" : 6.0,
									"-[34]" : 0.005354330708661,
									"-[35]" : 0.002047244094488,
									"-[36]" : 0.00251968503937,
									"-[37]" : 0.003464566929134,
									"-[38]" : 4.0,
									"-[39]" : 0.00251968503937,
									"-[3]" : 17.0,
									"-[40]" : 0.0,
									"-[41]" : 0.003464566929134,
									"-[42]" : 10.0,
									"-[43]" : 16.0,
									"-[44]" : 3.0,
									"-[45]" : 2.0,
									"-[46]" : 8.0,
									"-[47]" : 1.0,
									"-[48]" : 0.013385826771654,
									"-[49]" : 14.0,
									"-[4]" : 15.0,
									"-[50]" : -0.000314960629921,
									"-[51]" : 12.0,
									"-[52]" : 3.0,
									"-[53]" : 0.002834645669291,
									"-[54]" : 0.000314960629921,
									"-[55]" : 10.0,
									"-[56]" : 2.0,
									"-[57]" : 0.000787401574803,
									"-[58]" : 0.001259842519685,
									"-[59]" : 8.0,
									"-[5]" : 9.0,
									"-[60]" : 1.0,
									"-[61]" : 0.00755905511811,
									"-[62]" : 6.0,
									"-[63]" : 0.003527559055118,
									"-[6]" : 8.0,
									"-[7]" : 7.0,
									"-[8]" : 0.012913385826771,
									"-[9]" : 0.001889763779528,
									"button" : 0.0,
									"button[1]" : 0.0,
									"button[2]" : 0.0,
									"button[3]" : 0.0,
									"button[4]" : 0.0,
									"button[5]" : 0.0,
									"button[7]" : 0.0,
									"button[8]" : 0.0,
									"live.button[10]" : 0.0,
									"live.button[11]" : 0.0,
									"live.button[12]" : 0.0,
									"live.button[13]" : 0.0,
									"live.button[14]" : 0.0,
									"live.button[15]" : 0.0,
									"live.button[16]" : 0.0,
									"live.button[17]" : 0.0,
									"live.button[22]" : 0.0,
									"live.button[23]" : 0.0,
									"live.button[24]" : 0.0,
									"live.button[25]" : 0.0,
									"live.button[26]" : 0.0,
									"live.button[27]" : 0.0,
									"live.button[28]" : 0.0,
									"live.button[29]" : 0.0,
									"live.button[2]" : 0.0,
									"live.button[30]" : 0.0,
									"live.button[31]" : 0.0,
									"live.button[32]" : 0.0,
									"live.button[33]" : 0.0,
									"live.button[34]" : 0.0,
									"live.button[35]" : 0.0,
									"live.button[36]" : 0.0,
									"live.button[37]" : 0.0,
									"live.button[38]" : 0.0,
									"live.button[39]" : 0.0,
									"live.button[3]" : 0.0,
									"live.button[40]" : 0.0,
									"live.button[41]" : 0.0,
									"live.button[42]" : 0.0,
									"live.button[43]" : 0.0,
									"live.button[44]" : 0.0,
									"live.button[45]" : 0.0,
									"live.button[46]" : 0.0,
									"live.button[47]" : 0.0,
									"live.button[48]" : 0.0,
									"live.button[49]" : 0.0,
									"live.button[4]" : 0.0,
									"live.button[50]" : 0.0,
									"live.button[51]" : 0.0,
									"live.button[52]" : 0.0,
									"live.button[53]" : 0.0,
									"live.button[54]" : 0.0,
									"live.button[55]" : 0.0,
									"live.button[56]" : 0.0,
									"live.button[57]" : 0.0,
									"live.button[58]" : 0.0,
									"live.button[59]" : 0.0,
									"live.button[5]" : 0.0,
									"live.button[60]" : 0.0,
									"live.button[61]" : 0.0,
									"live.button[62]" : 0.0,
									"live.button[63]" : 0.0,
									"live.button[64]" : 0.0,
									"live.button[65]" : 0.0,
									"live.button[66]" : 0.0,
									"live.button[67]" : 0.0,
									"live.button[68]" : 0.0,
									"live.button[69]" : 0.0,
									"live.button[6]" : 0.0,
									"live.button[7]" : 0.0,
									"live.button[8]" : 0.0,
									"live.button[9]" : 0.0,
									"live.dial" : 275.275590551180642,
									"live.dial[10]" : 95.883958581172976,
									"live.dial[11]" : 1000.0,
									"live.dial[1]" : 58.487622929402356,
									"live.dial[2]" : 301.080681280403155,
									"live.dial[3]" : 231.819921000817317,
									"live.dial[4]" : 843.999999999999432,
									"live.dial[5]" : 3824.999999999999545,
									"live.dial[6]" : 285.999999999999943,
									"live.dial[7]" : 119.03844021206146,
									"live.dial[8]" : 3021.999999999999091,
									"live.dial[9]" : 323.757846634733824,
									"live.gain~" : 0.0,
									"live.menu" : 1.0,
									"live.menu[1]" : 0.0,
									"live.menu[2]" : 1.0,
									"live.menu[3]" : 3.0,
									"live.menu[4]" : 1.0,
									"live.menu[5]" : 2.0,
									"live.menu[6]" : 1.0,
									"live.menu[7]" : 1.0,
									"live.numbox" : 0.0,
									"live.numbox[1]" : 0.070866141732283,
									"live.numbox[2]" : 0.0,
									"live.numbox[3]" : 0.0,
									"mc.live.gain~[1]" : 0.0,
									"mc.live.gain~[3]" : 0.0,
									"mc.live.gain~[4]" : 0.0,
									"mc.live.gain~[5]" : 0.0,
									"blob" : 									{
										"flonum[1]" : [ 0.5 ],
										"flonum[2]" : [ 0.5 ],
										"flonum[3]" : [ 1.0 ],
										"flonum[4]" : [ 1.0 ],
										"flonum[5]" : [ 0.5 ],
										"flonum[6]" : [ 1.0 ],
										"flonum[7]" : [ 0.5 ],
										"flonum[8]" : [ 1.0 ],
										"live.grid[1]" : [ 3, 16, 8, 0, 23, 7, 1004, 2006, 2007, 3004, 4005, 4007, 5004, 6006, 6007, 7004, 8005, 8007, 9004, 10006, 10007, 11004, 12005, 12007, 13004, 14006, 14007, 15005, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 ],
										"live.grid[2]" : [ 3, 16, 8, 0, 6, 4, 3000, 8003, 12002, 13002, 15005, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 ],
										"live.grid[5]" : [ 3, 16, 8, 0, 11, 0, 1000, 3004, 4004, 6003, 7003, 9002, 10002, 12005, 13005, 15001, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 ],
										"live.grid[6]" : [ 3, 16, 8, 0, 8, 7, 2005, 5006, 6004, 9003, 11000, 13002, 14001, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 ],
										"multislider" : [ 0.454648974633986, 0.691840620963804, 0.729791284376575, 0.910056935587237, 0.957495264853201, 0.948007599000008, 0.938519933146815, 0.976470596559586 ],
										"multislider[1]" : [ 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 ],
										"multislider[2]" : [ 0.511574969753142, 0.644402291697841, 0.872106272174466, 1.0, 1.0, 1.0, 1.0, 1.0 ],
										"multislider[3]" : [ 0.577988630725491, 0.729791284376575, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 ],
										"number[10]" : [ 100 ],
										"number[11]" : [ 1000 ],
										"number[12]" : [ 1288 ],
										"number[13]" : [ 100 ],
										"number[14]" : [ 303 ],
										"number[15]" : [ 100 ],
										"number[16]" : [ 10492 ],
										"number[17]" : [ 200 ],
										"number[18]" : [ 600 ],
										"number[19]" : [ 100 ],
										"number[20]" : [ 1000 ],
										"number[21]" : [ 2581 ],
										"number[22]" : [ 100 ],
										"number[23]" : [ 615 ],
										"number[24]" : [ 600 ],
										"number[25]" : [ 100 ],
										"number[26]" : [ 1000 ],
										"number[27]" : [ 602 ],
										"number[28]" : [ 100 ],
										"number[29]" : [ 1052 ],
										"number[6]" : [ 1000 ],
										"number[7]" : [ 100 ],
										"number[8]" : [ 600 ],
										"number[9]" : [ 600 ]
									}

								}

							}

						}
,
						"fileref" : 						{
							"name" : "May2",
							"filename" : "jam1[1]_20260502.maxsnap",
							"filepath" : "~/Documents/Max 9/Snapshots",
							"filepos" : -1,
							"snapshotfileid" : "0a74361704a667b20853bb532de8bdd7"
						}

					}
, 					{
						"filetype" : "C74Snapshot",
						"version" : 2,
						"minorversion" : 0,
						"name" : "May3",
						"origin" : "MCSketch",
						"type" : "patcher",
						"subtype" : "Undefined",
						"embed" : 0,
						"fileref" : 						{
							"name" : "May3",
							"filename" : "jam1[1]_20260503.maxsnap",
							"filepath" : "~/Documents/Max 9/Snapshots",
							"filepos" : -1,
							"snapshotfileid" : "24785093c8084ece18f04bcb42ceb7b1"
						}

					}
, 					{
						"filetype" : "C74Snapshot",
						"version" : 2,
						"minorversion" : 0,
						"name" : "MASTER",
						"origin" : "MCSketch",
						"type" : "patcher",
						"subtype" : "Undefined",
						"embed" : 0,
						"fileref" : 						{
							"name" : "MASTER",
							"filename" : "May3[1].maxsnap",
							"filepath" : "~/Documents/Max 9/Snapshots",
							"filepos" : -1,
							"snapshotfileid" : "9fdea552b9daedc3c290213773cdfb84"
						}

					}
, 					{
						"filetype" : "C74Snapshot",
						"version" : 2,
						"minorversion" : 0,
						"name" : "dialTest",
						"origin" : "MCSketch",
						"type" : "patcher",
						"subtype" : "Undefined",
						"embed" : 0,
						"snapshot" : 						{
							"valuedictionary" : 							{
								"parameter_values" : 								{
									"-" : 1.0,
									"-[10]" : -0.004881889763779,
									"-[11]" : -0.004503937007874,
									"-[12]" : -0.00148031496063,
									"-[13]" : -0.00755905511811,
									"-[14]" : -0.008661417322835,
									"-[15]" : -0.01244094488189,
									"-[16]" : 6.0,
									"-[17]" : 3.0,
									"-[18]" : 0.004724409448819,
									"-[19]" : -0.001574803149606,
									"-[1]" : 3.0,
									"-[20]" : -0.002834645669291,
									"-[21]" : -0.00044094488189,
									"-[22]" : 0.0,
									"-[23]" : 12.0,
									"-[24]" : 12.0,
									"-[25]" : -0.0,
									"-[26]" : 15.0,
									"-[27]" : 0.002204724409449,
									"-[28]" : 10.0,
									"-[29]" : 15.0,
									"-[2]" : 5.0,
									"-[30]" : -0.000629921259843,
									"-[31]" : 5.0,
									"-[32]" : 0.006141732283465,
									"-[33]" : 6.0,
									"-[34]" : 0.001574803149607,
									"-[35]" : -0.002677165354331,
									"-[36]" : -0.004094488188976,
									"-[37]" : 0.000629921259842,
									"-[38]" : 4.0,
									"-[39]" : 0.001102362204724,
									"-[3]" : 17.0,
									"-[40]" : 0.0,
									"-[41]" : -0.00251968503937,
									"-[42]" : 10.0,
									"-[43]" : 16.0,
									"-[44]" : 3.0,
									"-[45]" : 5.0,
									"-[46]" : 8.0,
									"-[47]" : 1.0,
									"-[48]" : 0.012125984251969,
									"-[49]" : 14.0,
									"-[4]" : 15.0,
									"-[50]" : -0.01392125984252,
									"-[51]" : 12.0,
									"-[52]" : 3.0,
									"-[53]" : -0.003779527559055,
									"-[54]" : -0.001259842519686,
									"-[55]" : 10.0,
									"-[56]" : 2.0,
									"-[57]" : -0.002362204724409,
									"-[58]" : 0.0,
									"-[59]" : 7.0,
									"-[5]" : 9.0,
									"-[60]" : 1.0,
									"-[61]" : 0.003464566929134,
									"-[62]" : 6.0,
									"-[63]" : -0.000251968503937,
									"-[6]" : 8.0,
									"-[7]" : 7.0,
									"-[8]" : 0.008031496062992,
									"-[9]" : -0.005354330708661,
									"2" : 122.0,
									"2[10]" : 156.0,
									"2[11]" : 377.0,
									"2[1]" : 44.0,
									"2[2]" : 446.0,
									"2[3]" : 79.0,
									"2[4]" : 434.0,
									"2[5]" : 546.0,
									"2[6]" : 195.0,
									"2[7]" : 178.0,
									"2[8]" : 0.0,
									"2[9]" : 4.0,
									"button" : 0.0,
									"button[10]" : 0.0,
									"button[11]" : 0.0,
									"button[12]" : 0.0,
									"button[1]" : 0.0,
									"button[2]" : 0.0,
									"button[3]" : 0.0,
									"button[4]" : 0.0,
									"button[5]" : 0.0,
									"button[7]" : 0.0,
									"button[8]" : 0.0,
									"button[9]" : 0.0,
									"live.button[10]" : 0.0,
									"live.button[11]" : 0.0,
									"live.button[12]" : 0.0,
									"live.button[13]" : 0.0,
									"live.button[14]" : 0.0,
									"live.button[15]" : 0.0,
									"live.button[16]" : 0.0,
									"live.button[17]" : 0.0,
									"live.button[22]" : 0.0,
									"live.button[23]" : 0.0,
									"live.button[24]" : 0.0,
									"live.button[25]" : 1.0,
									"live.button[26]" : 0.0,
									"live.button[27]" : 0.0,
									"live.button[28]" : 0.0,
									"live.button[29]" : 0.0,
									"live.button[2]" : 0.0,
									"live.button[30]" : 0.0,
									"live.button[31]" : 0.0,
									"live.button[32]" : 0.0,
									"live.button[33]" : 0.0,
									"live.button[34]" : 0.0,
									"live.button[35]" : 0.0,
									"live.button[36]" : 0.0,
									"live.button[37]" : 0.0,
									"live.button[38]" : 0.0,
									"live.button[39]" : 0.0,
									"live.button[3]" : 0.0,
									"live.button[40]" : 0.0,
									"live.button[41]" : 0.0,
									"live.button[42]" : 0.0,
									"live.button[43]" : 0.0,
									"live.button[44]" : 0.0,
									"live.button[45]" : 0.0,
									"live.button[46]" : 0.0,
									"live.button[47]" : 0.0,
									"live.button[48]" : 0.0,
									"live.button[49]" : 0.0,
									"live.button[4]" : 0.0,
									"live.button[50]" : 0.0,
									"live.button[51]" : 0.0,
									"live.button[52]" : 0.0,
									"live.button[53]" : 0.0,
									"live.button[54]" : 0.0,
									"live.button[55]" : 0.0,
									"live.button[56]" : 0.0,
									"live.button[57]" : 0.0,
									"live.button[58]" : 0.0,
									"live.button[59]" : 0.0,
									"live.button[5]" : 0.0,
									"live.button[60]" : 0.0,
									"live.button[61]" : 0.0,
									"live.button[62]" : 0.0,
									"live.button[63]" : 0.0,
									"live.button[64]" : 0.0,
									"live.button[65]" : 0.0,
									"live.button[66]" : 0.0,
									"live.button[67]" : 0.0,
									"live.button[68]" : 0.0,
									"live.button[69]" : 0.0,
									"live.button[6]" : 0.0,
									"live.button[7]" : 0.0,
									"live.button[8]" : 0.0,
									"live.button[9]" : 0.0,
									"live.dial" : 275.275590551180642,
									"live.dial[10]" : 95.883958581172976,
									"live.dial[11]" : 1000.0,
									"live.dial[1]" : 57.983685921528291,
									"live.dial[2]" : 301.080681280403155,
									"live.dial[3]" : 231.315983992943302,
									"live.dial[4]" : 843.999999999999432,
									"live.dial[5]" : 3824.999999999999545,
									"live.dial[6]" : 285.999999999999943,
									"live.dial[7]" : 119.038440212061417,
									"live.dial[8]" : 3021.999999999999091,
									"live.dial[9]" : 323.757846634733824,
									"live.gain~" : 0.0,
									"live.menu" : 1.0,
									"live.menu[1]" : 0.0,
									"live.menu[2]" : 1.0,
									"live.menu[3]" : 3.0,
									"live.menu[4]" : 1.0,
									"live.menu[5]" : 2.0,
									"live.menu[6]" : 1.0,
									"live.menu[7]" : 1.0,
									"live.numbox" : 0.0,
									"live.numbox[1]" : 0.0,
									"live.numbox[2]" : 0.0,
									"live.numbox[3]" : 0.0,
									"live.text[10]" : 0.0,
									"live.text[11]" : 0.0,
									"live.text[12]" : 0.0,
									"live.text[13]" : 0.0,
									"live.text[14]" : 0.0,
									"live.text[15]" : 0.0,
									"live.text[16]" : 0.0,
									"live.text[17]" : 0.0,
									"live.text[18]" : 0.0,
									"live.text[19]" : 0.0,
									"live.text[1]" : 0.0,
									"live.text[20]" : 0.0,
									"live.text[24]" : 0.0,
									"live.text[28]" : 0.0,
									"live.text[29]" : 0.0,
									"live.text[2]" : 0.0,
									"live.text[30]" : 0.0,
									"live.text[31]" : 0.0,
									"live.text[32]" : 0.0,
									"live.text[33]" : 0.0,
									"live.text[34]" : 0.0,
									"live.text[3]" : 0.0,
									"live.text[4]" : 0.0,
									"live.text[5]" : 0.0,
									"live.text[6]" : 0.0,
									"live.text[70]" : 0.0,
									"live.text[71]" : 0.0,
									"live.text[78]" : 0.0,
									"live.text[79]" : 0.0,
									"live.text[7]" : 0.0,
									"live.text[8]" : 0.0,
									"live.text[9]" : 0.0,
									"mc.live.gain~[1]" : 0.0,
									"mc.live.gain~[3]" : 0.0,
									"mc.live.gain~[4]" : 0.0,
									"mc.live.gain~[5]" : 0.0,
									"pushtext16[1]" : 0.0,
									"pushtext18[1]" : 0.0,
									"pushtext20[1]" : 0.0,
									"pushtext22[1]" : 1.0,
									"blob" : 									{
										"flonum[1]" : [ 0.5 ],
										"flonum[2]" : [ 0.5 ],
										"flonum[3]" : [ 1.0 ],
										"flonum[4]" : [ 1.0 ],
										"flonum[5]" : [ 0.5 ],
										"flonum[6]" : [ 1.0 ],
										"flonum[7]" : [ 0.5 ],
										"flonum[8]" : [ 1.0 ],
										"live.grid[1]" : [ 3, 16, 8, 0, 23, 7, 1004, 2006, 2007, 3004, 4005, 4007, 5004, 6006, 6007, 7004, 8005, 8007, 9004, 10006, 10007, 11004, 12005, 12007, 13004, 14006, 14007, 15005, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 ],
										"live.grid[2]" : [ 3, 16, 8, 0, 6, 4, 3000, 8003, 12002, 13002, 15005, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 ],
										"live.grid[5]" : [ 3, 16, 8, 0, 11, 0, 1000, 3004, 4004, 6003, 7003, 9002, 10002, 12005, 13005, 15001, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 ],
										"live.grid[6]" : [ 3, 16, 8, 0, 8, 7, 2005, 5006, 6004, 9003, 11000, 13002, 14001, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 ],
										"multislider" : [ 0.594930883376829, 1.0, 1.0, 1.0, 0.957495264853201, 0.948007599000008, 1.0, 1.0 ],
										"multislider[1]" : [ 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 ],
										"multislider[2]" : [ 0.587476296578684, 0.825345627723202, 0.606451620594148, 1.0, 0.900569269734044, 1.0, 0.894470051027113, 1.0 ],
										"multislider[3]" : [ 0.530550301459528, 0.682352955110611, 0.952073737113707, 0.698617518332697, 0.617972357811466, 1.0, 1.0, 1.0 ],
										"number[10]" : [ 100 ],
										"number[11]" : [ 1000 ],
										"number[12]" : [ 1288 ],
										"number[13]" : [ 100 ],
										"number[14]" : [ 303 ],
										"number[15]" : [ 100 ],
										"number[16]" : [ 40000 ],
										"number[17]" : [ 1000 ],
										"number[18]" : [ 600 ],
										"number[19]" : [ 100 ],
										"number[20]" : [ 1000 ],
										"number[21]" : [ 2581 ],
										"number[22]" : [ 100 ],
										"number[23]" : [ 615 ],
										"number[24]" : [ 600 ],
										"number[25]" : [ 100 ],
										"number[26]" : [ 1000 ],
										"number[27]" : [ 602 ],
										"number[28]" : [ 100 ],
										"number[29]" : [ 1052 ],
										"number[6]" : [ 1000 ],
										"number[7]" : [ 100 ],
										"number[8]" : [ 600 ],
										"number[9]" : [ 600 ]
									}

								}

							}

						}
,
						"fileref" : 						{
							"name" : "dialTest",
							"filename" : "MASTER[1].maxsnap",
							"filepath" : "~/Documents/Max 9/Snapshots",
							"filepos" : -1,
							"snapshotfileid" : "48dce6eb3c58ae620bfdd1b5488bbec2"
						}

					}
, 					{
						"filetype" : "C74Snapshot",
						"version" : 2,
						"minorversion" : 0,
						"name" : "May4",
						"origin" : "MCSketch",
						"type" : "patcher",
						"subtype" : "Undefined",
						"embed" : 0,
						"fileref" : 						{
							"name" : "May4",
							"filename" : "MASTER[1]_20260504.maxsnap",
							"filepath" : "~/Documents/Max 9/Snapshots",
							"filepos" : -1,
							"snapshotfileid" : "e9438de476623ecdd240715e479f1906"
						}

					}
, 					{
						"filetype" : "C74Snapshot",
						"version" : 2,
						"minorversion" : 0,
						"name" : "May7",
						"origin" : "MCSketch",
						"type" : "patcher",
						"subtype" : "Undefined",
						"embed" : 0,
						"snapshot" : 						{
							"valuedictionary" : 							{
								"parameter_values" : 								{
									"-" : 1.0,
									"-[10]" : -0.0002,
									"-[11]" : -0.00164,
									"-[12]" : 0.01232,
									"-[13]" : -0.00064,
									"-[14]" : -0.00224,
									"-[15]" : -0.0066,
									"-[16]" : 6.0,
									"-[17]" : 3.0,
									"-[18]" : 0.00332,
									"-[19]" : 0.00352,
									"-[1]" : 3.0,
									"-[20]" : 0.00048,
									"-[21]" : 0.0022,
									"-[22]" : 0.00128,
									"-[23]" : 12.0,
									"-[24]" : 12.0,
									"-[25]" : 0.00496,
									"-[26]" : 15.0,
									"-[27]" : 0.00424,
									"-[28]" : 10.0,
									"-[29]" : 15.0,
									"-[2]" : 5.0,
									"-[30]" : 0.00056,
									"-[31]" : 5.0,
									"-[32]" : 0.00316,
									"-[33]" : 6.0,
									"-[34]" : -0.00056,
									"-[35]" : -0.00396,
									"-[36]" : -0.00336,
									"-[37]" : -0.00496,
									"-[38]" : 4.0,
									"-[39]" : -0.002,
									"-[3]" : 14.0,
									"-[40]" : -0.005,
									"-[41]" : -0.00592,
									"-[42]" : 10.0,
									"-[43]" : 16.0,
									"-[44]" : 3.0,
									"-[45]" : 5.0,
									"-[46]" : 8.0,
									"-[47]" : 2.0,
									"-[48]" : 0.01288,
									"-[49]" : 14.0,
									"-[4]" : 15.0,
									"-[50]" : -0.00604,
									"-[51]" : 12.0,
									"-[52]" : 3.0,
									"-[53]" : -0.00176,
									"-[54]" : 0.0,
									"-[55]" : 10.0,
									"-[56]" : 4.0,
									"-[57]" : 0.00004,
									"-[58]" : 0.0,
									"-[59]" : 7.0,
									"-[5]" : 9.0,
									"-[60]" : 2.0,
									"-[61]" : 0.00668,
									"-[62]" : 6.0,
									"-[63]" : 0.00092,
									"-[6]" : 8.0,
									"-[7]" : 7.0,
									"-[8]" : 0.0118,
									"-[9]" : 0.0,
									"2" : 335.0,
									"2[10]" : 349.0,
									"2[11]" : 484.0,
									"2[12]" : 606.0,
									"2[13]" : 401.0,
									"2[14]" : 501.0,
									"2[15]" : 459.0,
									"2[16]" : 579.0,
									"2[17]" : 822.0,
									"2[18]" : 808.0,
									"2[19]" : 583.0,
									"2[1]" : 500.0,
									"2[20]" : 486.0,
									"2[21]" : 667.0,
									"2[22]" : 795.0,
									"2[23]" : 512.0,
									"2[24]" : 450.0,
									"2[25]" : 523.0,
									"2[26]" : 500.0,
									"2[27]" : 588.0,
									"2[28]" : 352.0,
									"2[29]" : 456.0,
									"2[2]" : 375.0,
									"2[30]" : 555.0,
									"2[31]" : 495.0,
									"2[3]" : 514.0,
									"2[4]" : 532.0,
									"2[5]" : 376.0,
									"2[6]" : 500.0,
									"2[7]" : 444.0,
									"2[8]" : 624.0,
									"2[9]" : 416.0,
									"button" : 0.0,
									"button[1]" : 0.0,
									"button[2]" : 0.0,
									"button[3]" : 0.0,
									"button[4]" : 0.0,
									"button[5]" : 0.0,
									"button[7]" : 0.0,
									"button[8]" : 0.0,
									"live.button[10]" : 0.0,
									"live.button[11]" : 0.0,
									"live.button[12]" : 0.0,
									"live.button[13]" : 0.0,
									"live.button[14]" : 0.0,
									"live.button[15]" : 0.0,
									"live.button[16]" : 0.0,
									"live.button[17]" : 0.0,
									"live.button[22]" : 0.0,
									"live.button[23]" : 0.0,
									"live.button[24]" : 0.0,
									"live.button[25]" : 0.0,
									"live.button[26]" : 0.0,
									"live.button[27]" : 0.0,
									"live.button[28]" : 0.0,
									"live.button[29]" : 0.0,
									"live.button[2]" : 0.0,
									"live.button[30]" : 0.0,
									"live.button[31]" : 0.0,
									"live.button[32]" : 0.0,
									"live.button[33]" : 0.0,
									"live.button[34]" : 0.0,
									"live.button[35]" : 0.0,
									"live.button[36]" : 0.0,
									"live.button[37]" : 0.0,
									"live.button[38]" : 0.0,
									"live.button[39]" : 0.0,
									"live.button[3]" : 0.0,
									"live.button[40]" : 0.0,
									"live.button[41]" : 0.0,
									"live.button[42]" : 0.0,
									"live.button[43]" : 0.0,
									"live.button[44]" : 0.0,
									"live.button[45]" : 0.0,
									"live.button[46]" : 0.0,
									"live.button[47]" : 0.0,
									"live.button[48]" : 0.0,
									"live.button[49]" : 0.0,
									"live.button[4]" : 0.0,
									"live.button[50]" : 0.0,
									"live.button[51]" : 0.0,
									"live.button[52]" : 0.0,
									"live.button[53]" : 0.0,
									"live.button[54]" : 0.0,
									"live.button[55]" : 0.0,
									"live.button[56]" : 0.0,
									"live.button[57]" : 0.0,
									"live.button[58]" : 0.0,
									"live.button[59]" : 0.0,
									"live.button[5]" : 0.0,
									"live.button[60]" : 0.0,
									"live.button[61]" : 0.0,
									"live.button[62]" : 0.0,
									"live.button[63]" : 0.0,
									"live.button[64]" : 0.0,
									"live.button[65]" : 0.0,
									"live.button[66]" : 0.0,
									"live.button[67]" : 0.0,
									"live.button[68]" : 0.0,
									"live.button[69]" : 0.0,
									"live.button[6]" : 0.0,
									"live.button[7]" : 0.0,
									"live.button[8]" : 0.0,
									"live.button[9]" : 0.0,
									"live.dial[10]" : 93.868210549675425,
									"live.dial[11]" : 1000.0,
									"live.dial[1]" : 57.40617747688075,
									"live.dial[3]" : 230.523213011903039,
									"live.dial[4]" : 843.999999999999432,
									"live.dial[5]" : 4000.0,
									"live.dial[7]" : 118.534503204186876,
									"live.dial[8]" : 3021.999999999999091,
									"live.menu" : 1.0,
									"live.menu[1]" : 0.0,
									"live.menu[2]" : 1.0,
									"live.menu[3]" : 3.0,
									"live.menu[4]" : 1.0,
									"live.menu[5]" : 2.0,
									"live.menu[6]" : 1.0,
									"live.menu[7]" : 1.0,
									"live.numbox" : 0.0,
									"live.numbox[1]" : 0.0,
									"live.numbox[2]" : 0.0,
									"live.numbox[3]" : 0.0,
									"live.text[10]" : 0.0,
									"live.text[11]" : 0.0,
									"live.text[12]" : 0.0,
									"live.text[13]" : 0.0,
									"live.text[14]" : 0.0,
									"live.text[15]" : 0.0,
									"live.text[16]" : 0.0,
									"live.text[17]" : 0.0,
									"live.text[18]" : 0.0,
									"live.text[19]" : 0.0,
									"live.text[1]" : 0.0,
									"live.text[20]" : 0.0,
									"live.text[24]" : 0.0,
									"live.text[28]" : 0.0,
									"live.text[29]" : 0.0,
									"live.text[2]" : 0.0,
									"live.text[30]" : 0.0,
									"live.text[31]" : 0.0,
									"live.text[32]" : 0.0,
									"live.text[33]" : 0.0,
									"live.text[34]" : 0.0,
									"live.text[3]" : 0.0,
									"live.text[4]" : 0.0,
									"live.text[5]" : 0.0,
									"live.text[6]" : 0.0,
									"live.text[70]" : 0.0,
									"live.text[71]" : 0.0,
									"live.text[78]" : 0.0,
									"live.text[79]" : 0.0,
									"live.text[7]" : 0.0,
									"live.text[8]" : 0.0,
									"live.text[9]" : 0.0,
									"mc.live.gain~[1]" : 0.0,
									"mc.live.gain~[3]" : 0.0,
									"mc.live.gain~[4]" : 0.0,
									"mc.live.gain~[5]" : 0.0,
									"pushtext16[1]" : 1.0,
									"pushtext18[1]" : 0.0,
									"pushtext20[1]" : 0.0,
									"pushtext22[1]" : 0.0,
									"pushtext2[1]" : 0.0,
									"pushtext3[1]" : 0.0,
									"pushtext4[1]" : 0.0,
									"pushtext5[1]" : 0.0,
									"blob" : 									{
										"flonum[1]" : [ 0.5 ],
										"flonum[2]" : [ 0.5 ],
										"flonum[3]" : [ 1.0 ],
										"flonum[4]" : [ 1.0 ],
										"flonum[5]" : [ 0.5 ],
										"flonum[6]" : [ 1.0 ],
										"flonum[7]" : [ 0.5 ],
										"flonum[8]" : [ 1.0 ],
										"live.grid[1]" : [ 3, 16, 8, 0, 24, 7, 1004, 2006, 2007, 3004, 4005, 4007, 5002, 5003, 6006, 6007, 7004, 8005, 8007, 9004, 10006, 10007, 11001, 12005, 12007, 13004, 14006, 14007, 15005, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 ],
										"live.grid[2]" : [ 3, 16, 8, 0, 6, 4, 3000, 8003, 12002, 13002, 15005, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 ],
										"live.grid[5]" : [ 3, 16, 8, 0, 11, 0, 1000, 3004, 4004, 6003, 7003, 9002, 10002, 12005, 13005, 15001, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 ],
										"live.grid[6]" : [ 3, 16, 8, 0, 8, 7, 2005, 5006, 6004, 9003, 11000, 13002, 14001, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 ],
										"multislider" : [ 0.592626708553683, 0.850691230835453, 0.88755759116142, 0.859907820916945, 0.915207361405896, 0.933640541568879, 0.979723491976338, 0.952073721731863 ],
										"multislider[1]" : [ 1.0, 1.0, 1.0, 1.0, 0.915207361405896, 1.0, 1.0, 0.869124410998437 ],
										"multislider[2]" : [ 0.88755759116142, 0.915207361405896, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 ],
										"multislider[3]" : [ 0.749308739939044, 0.749308739939044, 0.749308739939044, 1.0, 0.684792609368601, 1.0, 1.0, 1.0 ],
										"number[10]" : [ 100 ],
										"number[11]" : [ 1000 ],
										"number[12]" : [ 1288 ],
										"number[13]" : [ 100 ],
										"number[14]" : [ 303 ],
										"number[15]" : [ 100 ],
										"number[16]" : [ 40000 ],
										"number[17]" : [ 1000 ],
										"number[18]" : [ 600 ],
										"number[19]" : [ 100 ],
										"number[20]" : [ 1000 ],
										"number[21]" : [ 2581 ],
										"number[22]" : [ 100 ],
										"number[23]" : [ 615 ],
										"number[24]" : [ 600 ],
										"number[25]" : [ 100 ],
										"number[26]" : [ 1000 ],
										"number[27]" : [ 602 ],
										"number[28]" : [ 100 ],
										"number[29]" : [ 1052 ],
										"number[6]" : [ 1000 ],
										"number[7]" : [ 100 ],
										"number[8]" : [ 600 ],
										"number[9]" : [ 600 ]
									}

								}

							}

						}
,
						"fileref" : 						{
							"name" : "May7",
							"filename" : "jam1[1]_20260507.maxsnap",
							"filepath" : "~/Documents/Max 9/Snapshots",
							"filepos" : -1,
							"snapshotfileid" : "d83c30a809fd54980477ec601eaae236"
						}

					}
, 					{
						"filetype" : "C74Snapshot",
						"version" : 2,
						"minorversion" : 0,
						"name" : "May9",
						"origin" : "MCSketch",
						"type" : "patcher",
						"subtype" : "Undefined",
						"embed" : 0,
						"snapshot" : 						{
							"valuedictionary" : 							{
								"parameter_values" : 								{
									"-" : 1.0,
									"-[10]" : -0.00512,
									"-[11]" : -0.00756,
									"-[12]" : 0.00496,
									"-[13]" : -0.00612,
									"-[14]" : -0.00016,
									"-[15]" : -0.01504,
									"-[16]" : 3.0,
									"-[17]" : 2.0,
									"-[18]" : 0.00436,
									"-[19]" : 0.00248,
									"-[1]" : 3.0,
									"-[20]" : 0.00268,
									"-[21]" : 0.00024,
									"-[22]" : 0.0026,
									"-[23]" : 6.0,
									"-[24]" : 5.0,
									"-[25]" : 0.0036,
									"-[26]" : 15.0,
									"-[27]" : 0.003,
									"-[28]" : 10.0,
									"-[29]" : 15.0,
									"-[2]" : 5.0,
									"-[30]" : 0.0028,
									"-[31]" : 10.0,
									"-[32]" : -0.00408,
									"-[33]" : 6.0,
									"-[34]" : -0.00408,
									"-[35]" : -0.0076,
									"-[36]" : 0.00016,
									"-[37]" : 0.00188,
									"-[38]" : 4.0,
									"-[39]" : -0.00556,
									"-[3]" : 14.0,
									"-[40]" : 0.00292,
									"-[41]" : -0.00744,
									"-[42]" : 10.0,
									"-[43]" : 16.0,
									"-[44]" : 3.0,
									"-[45]" : 5.0,
									"-[46]" : 8.0,
									"-[47]" : 2.0,
									"-[48]" : 0.00108,
									"-[49]" : 14.0,
									"-[4]" : 15.0,
									"-[50]" : -0.00316,
									"-[51]" : 12.0,
									"-[52]" : 3.0,
									"-[53]" : -0.00176,
									"-[54]" : 0.0,
									"-[55]" : 10.0,
									"-[56]" : 4.0,
									"-[57]" : -0.00592,
									"-[58]" : 0.0,
									"-[59]" : 7.0,
									"-[5]" : 9.0,
									"-[60]" : 2.0,
									"-[61]" : -0.00512,
									"-[62]" : 6.0,
									"-[63]" : -0.00832,
									"-[6]" : 8.0,
									"-[7]" : 7.0,
									"-[8]" : 0.00796,
									"-[9]" : -0.0026,
									"2" : 124.0,
									"2[10]" : 421.0,
									"2[11]" : 347.0,
									"2[12]" : 575.0,
									"2[13]" : 310.0,
									"2[14]" : 352.0,
									"2[15]" : 311.0,
									"2[16]" : 398.0,
									"2[17]" : 527.0,
									"2[18]" : 624.0,
									"2[19]" : 609.0,
									"2[1]" : 500.0,
									"2[20]" : 398.0,
									"2[21]" : 372.0,
									"2[22]" : 699.0,
									"2[23]" : 567.0,
									"2[24]" : 361.0,
									"2[25]" : 292.0,
									"2[26]" : 435.0,
									"2[27]" : 562.0,
									"2[28]" : 314.0,
									"2[29]" : 456.0,
									"2[2]" : 573.0,
									"2[30]" : 506.0,
									"2[31]" : 372.0,
									"2[3]" : 570.0,
									"2[4]" : 565.0,
									"2[5]" : 547.0,
									"2[6]" : 500.0,
									"2[7]" : 496.0,
									"2[8]" : 590.0,
									"2[9]" : 504.0,
									"button" : 0.0,
									"button[1]" : 0.0,
									"button[2]" : 0.0,
									"button[3]" : 0.0,
									"button[4]" : 0.0,
									"button[5]" : 0.0,
									"button[7]" : 0.0,
									"button[8]" : 0.0,
									"live.button[10]" : 0.0,
									"live.button[11]" : 0.0,
									"live.button[12]" : 0.0,
									"live.button[13]" : 0.0,
									"live.button[14]" : 0.0,
									"live.button[15]" : 0.0,
									"live.button[16]" : 0.0,
									"live.button[17]" : 0.0,
									"live.button[22]" : 0.0,
									"live.button[23]" : 0.0,
									"live.button[24]" : 0.0,
									"live.button[25]" : 0.0,
									"live.button[26]" : 0.0,
									"live.button[27]" : 0.0,
									"live.button[28]" : 0.0,
									"live.button[29]" : 0.0,
									"live.button[2]" : 0.0,
									"live.button[30]" : 0.0,
									"live.button[31]" : 0.0,
									"live.button[32]" : 0.0,
									"live.button[33]" : 0.0,
									"live.button[34]" : 0.0,
									"live.button[35]" : 0.0,
									"live.button[36]" : 0.0,
									"live.button[37]" : 0.0,
									"live.button[38]" : 0.0,
									"live.button[39]" : 0.0,
									"live.button[3]" : 0.0,
									"live.button[40]" : 0.0,
									"live.button[41]" : 0.0,
									"live.button[42]" : 0.0,
									"live.button[43]" : 0.0,
									"live.button[44]" : 0.0,
									"live.button[45]" : 0.0,
									"live.button[46]" : 0.0,
									"live.button[47]" : 0.0,
									"live.button[48]" : 0.0,
									"live.button[49]" : 0.0,
									"live.button[4]" : 0.0,
									"live.button[50]" : 0.0,
									"live.button[51]" : 0.0,
									"live.button[52]" : 0.0,
									"live.button[53]" : 0.0,
									"live.button[54]" : 0.0,
									"live.button[55]" : 0.0,
									"live.button[56]" : 0.0,
									"live.button[57]" : 0.0,
									"live.button[58]" : 0.0,
									"live.button[59]" : 0.0,
									"live.button[5]" : 0.0,
									"live.button[60]" : 0.0,
									"live.button[61]" : 0.0,
									"live.button[62]" : 0.0,
									"live.button[63]" : 0.0,
									"live.button[64]" : 0.0,
									"live.button[65]" : 0.0,
									"live.button[66]" : 0.0,
									"live.button[67]" : 0.0,
									"live.button[68]" : 0.0,
									"live.button[69]" : 0.0,
									"live.button[6]" : 0.0,
									"live.button[7]" : 0.0,
									"live.button[8]" : 0.0,
									"live.button[9]" : 0.0,
									"live.dial[10]" : 94.372147557548828,
									"live.dial[11]" : 1000.0,
									"live.dial[1]" : 55.894366453258264,
									"live.dial[3]" : 215.113016492931564,
									"live.dial[4]" : 843.999999999999432,
									"live.dial[5]" : 4000.0,
									"live.dial[7]" : 118.53450320418537,
									"live.dial[8]" : 3021.999999999999091,
									"live.gain~" : -14.423593579450323,
									"live.menu" : 1.0,
									"live.menu[1]" : 0.0,
									"live.menu[2]" : 1.0,
									"live.menu[3]" : 3.0,
									"live.menu[4]" : 1.0,
									"live.menu[5]" : 2.0,
									"live.menu[6]" : 1.0,
									"live.menu[7]" : 1.0,
									"live.numbox" : 0.0,
									"live.numbox[1]" : 0.0,
									"live.numbox[2]" : 0.0,
									"live.numbox[3]" : 0.0,
									"live.text[10]" : 0.0,
									"live.text[11]" : 0.0,
									"live.text[12]" : 0.0,
									"live.text[13]" : 0.0,
									"live.text[14]" : 0.0,
									"live.text[15]" : 0.0,
									"live.text[16]" : 0.0,
									"live.text[17]" : 0.0,
									"live.text[18]" : 0.0,
									"live.text[19]" : 0.0,
									"live.text[1]" : 0.0,
									"live.text[20]" : 0.0,
									"live.text[24]" : 0.0,
									"live.text[28]" : 0.0,
									"live.text[29]" : 0.0,
									"live.text[2]" : 0.0,
									"live.text[30]" : 0.0,
									"live.text[31]" : 0.0,
									"live.text[32]" : 0.0,
									"live.text[33]" : 0.0,
									"live.text[34]" : 0.0,
									"live.text[3]" : 0.0,
									"live.text[4]" : 0.0,
									"live.text[5]" : 0.0,
									"live.text[6]" : 0.0,
									"live.text[70]" : 0.0,
									"live.text[71]" : 0.0,
									"live.text[78]" : 0.0,
									"live.text[79]" : 0.0,
									"live.text[7]" : 0.0,
									"live.text[8]" : 0.0,
									"live.text[9]" : 0.0,
									"mc.live.gain~[1]" : 0.0,
									"mc.live.gain~[3]" : 0.0,
									"mc.live.gain~[4]" : 0.0,
									"mc.live.gain~[5]" : 0.0,
									"pushtext16[1]" : 0.0,
									"pushtext18[1]" : 0.0,
									"pushtext20[1]" : 0.0,
									"pushtext22[1]" : 1.0,
									"pushtext2[1]" : 0.0,
									"pushtext3[1]" : 0.0,
									"pushtext4[1]" : 0.0,
									"pushtext5[1]" : 0.0,
									"blob" : 									{
										"flonum[1]" : [ 0.5 ],
										"flonum[2]" : [ 0.5 ],
										"flonum[3]" : [ 1.0 ],
										"flonum[4]" : [ 1.0 ],
										"flonum[5]" : [ 0.5 ],
										"flonum[6]" : [ 1.0 ],
										"flonum[7]" : [ 0.5 ],
										"flonum[8]" : [ 1.0 ],
										"live.grid" : [ 0, 16, 16, 0, 16, 0, 1001, 2002, 3003, 4004, 5005, 6006, 7007, 8008, 9009, 10010, 11011, 12012, 13013, 14014, 15015, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 ],
										"live.grid[1]" : [ 3, 16, 8, 0, 24, 7, 1004, 2006, 2007, 3004, 4005, 4007, 5002, 5003, 6006, 6007, 7004, 8005, 8007, 9004, 10006, 10007, 11001, 12005, 12007, 13004, 14006, 14007, 15005, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 ],
										"live.grid[2]" : [ 3, 16, 8, 0, 6, 4, 3000, 8003, 12002, 13002, 15005, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 ],
										"live.grid[5]" : [ 3, 16, 8, 0, 11, 0, 1000, 3004, 4004, 6003, 7003, 9002, 10002, 12005, 13005, 15001, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 ],
										"live.grid[6]" : [ 3, 16, 8, 0, 8, 7, 2005, 5006, 6004, 9003, 11000, 13002, 14001, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 ],
										"multislider" : [ 0.682352955110611, 1.0, 0.88755759116142, 0.948007599000008, 1.0, 0.933640541568879, 0.979723491976338, 0.952073721731863 ],
										"multislider[1]" : [ 0.929032267293622, 0.929032267293622, 0.938519933146815, 0.948007599000008, 0.976470596559586, 1.0, 1.0, 1.0 ],
										"multislider[2]" : [ 0.966982930706393, 1.0, 0.881593938027659, 1.0, 1.0, 1.0, 1.0, 1.0 ],
										"multislider[3]" : [ 1.0, 0.749308739939044, 1.0, 1.0, 0.684792609368601, 1.0, 1.0, 1.0 ],
										"number[10]" : [ 100 ],
										"number[11]" : [ 1000 ],
										"number[12]" : [ 1288 ],
										"number[13]" : [ 100 ],
										"number[14]" : [ 303 ],
										"number[15]" : [ 100 ],
										"number[16]" : [ 50000 ],
										"number[17]" : [ 1695 ],
										"number[18]" : [ 600 ],
										"number[19]" : [ 100 ],
										"number[20]" : [ 1000 ],
										"number[21]" : [ 2581 ],
										"number[22]" : [ 100 ],
										"number[23]" : [ 615 ],
										"number[24]" : [ 600 ],
										"number[25]" : [ 100 ],
										"number[26]" : [ 1000 ],
										"number[27]" : [ 602 ],
										"number[28]" : [ 100 ],
										"number[29]" : [ 1052 ],
										"number[6]" : [ 1000 ],
										"number[7]" : [ 100 ],
										"number[8]" : [ 600 ],
										"number[9]" : [ 600 ]
									}

								}

							}

						}
,
						"fileref" : 						{
							"name" : "May9",
							"filename" : "jam1[1]_20260509.maxsnap",
							"filepath" : "~/Documents/Max 9/Snapshots",
							"filepos" : -1,
							"snapshotfileid" : "984a2ef1b9bd550c5f021f71aaef59b3"
						}

					}
, 					{
						"filetype" : "C74Snapshot",
						"version" : 2,
						"minorversion" : 0,
						"name" : "May10",
						"origin" : "MCSketch",
						"type" : "patcher",
						"subtype" : "Undefined",
						"embed" : 0,
						"fileref" : 						{
							"name" : "jam1[1]",
							"filename" : "jam1[1]_20260510.maxsnap",
							"filepath" : "~/Documents/Max 9/Snapshots",
							"filepos" : -1,
							"snapshotfileid" : "e29488647e205b36e20718de5c81e492"
						}

					}
, 					{
						"filetype" : "C74Snapshot",
						"version" : 2,
						"minorversion" : 0,
						"name" : "may21",
						"origin" : "MCSketch",
						"type" : "patcher",
						"subtype" : "Undefined",
						"embed" : 0,
						"fileref" : 						{
							"name" : "may21",
							"filename" : "jam1[1]_20260521.maxsnap",
							"filepath" : "~/Documents/Max 9/Snapshots",
							"filepos" : -1,
							"snapshotfileid" : "83616633c2244e25c577b7abfead4b00"
						}

					}
, 					{
						"filetype" : "C74Snapshot",
						"version" : 2,
						"minorversion" : 0,
						"name" : "cmmsd",
						"origin" : "MCSketch",
						"type" : "patcher",
						"subtype" : "Undefined",
						"embed" : 1,
						"snapshot" : 						{
							"valuedictionary" : 							{
								"parameter_values" : 								{
									"-" : 1.0,
									"-[10]" : 0.00032,
									"-[11]" : 0.00184,
									"-[12]" : 0.00484,
									"-[13]" : 0.00008,
									"-[14]" : -0.00152,
									"-[15]" : -0.01256,
									"-[16]" : 4.0,
									"-[17]" : 3.0,
									"-[18]" : 0.00556,
									"-[19]" : 0.00216,
									"-[1]" : 3.0,
									"-[20]" : 0.00412,
									"-[21]" : 0.0118,
									"-[22]" : 0.00056,
									"-[23]" : 5.0,
									"-[24]" : 6.0,
									"-[25]" : 0.00196,
									"-[26]" : 10.0,
									"-[27]" : 0.00292,
									"-[28]" : 9.0,
									"-[29]" : 15.0,
									"-[2]" : 5.0,
									"-[30]" : 0.0,
									"-[31]" : 5.0,
									"-[32]" : 0.00536,
									"-[33]" : 6.0,
									"-[34]" : 0.00236,
									"-[35]" : -0.00112,
									"-[36]" : -0.0098,
									"-[37]" : -0.00804,
									"-[38]" : 7.0,
									"-[39]" : -0.00228,
									"-[3]" : 14.0,
									"-[40]" : -0.00748,
									"-[41]" : -0.0084,
									"-[42]" : 10.0,
									"-[43]" : 11.0,
									"-[44]" : 3.0,
									"-[45]" : 5.0,
									"-[46]" : 8.0,
									"-[47]" : 1.0,
									"-[48]" : -0.00412,
									"-[49]" : 14.0,
									"-[4]" : 15.0,
									"-[50]" : 0.00312,
									"-[51]" : 9.0,
									"-[52]" : 3.0,
									"-[53]" : -0.00064,
									"-[54]" : 0.00444,
									"-[55]" : 8.0,
									"-[56]" : 5.0,
									"-[57]" : 0.0044,
									"-[58]" : 0.00436,
									"-[59]" : 6.0,
									"-[5]" : 11.0,
									"-[60]" : 1.0,
									"-[61]" : 0.01132,
									"-[62]" : 5.0,
									"-[63]" : 0.01064,
									"-[6]" : 9.0,
									"-[7]" : 7.0,
									"-[8]" : 0.00452,
									"-[9]" : 0.00512,
									"2" : 186.0,
									"2[10]" : 578.0,
									"2[11]" : 502.0,
									"2[12]" : 573.0,
									"2[13]" : 472.0,
									"2[14]" : 610.0,
									"2[15]" : 546.0,
									"2[16]" : 634.0,
									"2[17]" : 397.0,
									"2[18]" : 621.0,
									"2[19]" : 639.0,
									"2[1]" : 609.0,
									"2[20]" : 559.0,
									"2[21]" : 783.0,
									"2[22]" : 613.0,
									"2[23]" : 603.0,
									"2[24]" : 443.0,
									"2[25]" : 766.0,
									"2[26]" : 628.0,
									"2[27]" : 554.0,
									"2[28]" : 290.0,
									"2[29]" : 484.0,
									"2[2]" : 313.0,
									"2[30]" : 795.0,
									"2[31]" : 508.0,
									"2[3]" : 500.0,
									"2[4]" : 514.0,
									"2[5]" : 299.0,
									"2[6]" : 611.0,
									"2[7]" : 462.0,
									"2[8]" : 549.0,
									"2[9]" : 255.0,
									"button" : 0.0,
									"button[1]" : 0.0,
									"button[2]" : 0.0,
									"button[3]" : 0.0,
									"button[4]" : 0.0,
									"button[5]" : 0.0,
									"button[7]" : 0.0,
									"button[8]" : 0.0,
									"live.button[10]" : 0.0,
									"live.button[11]" : 0.0,
									"live.button[12]" : 0.0,
									"live.button[13]" : 0.0,
									"live.button[14]" : 0.0,
									"live.button[15]" : 0.0,
									"live.button[16]" : 0.0,
									"live.button[17]" : 0.0,
									"live.button[22]" : 0.0,
									"live.button[23]" : 0.0,
									"live.button[24]" : 0.0,
									"live.button[25]" : 0.0,
									"live.button[26]" : 0.0,
									"live.button[27]" : 0.0,
									"live.button[28]" : 0.0,
									"live.button[29]" : 0.0,
									"live.button[2]" : 0.0,
									"live.button[30]" : 0.0,
									"live.button[31]" : 0.0,
									"live.button[32]" : 0.0,
									"live.button[33]" : 0.0,
									"live.button[34]" : 0.0,
									"live.button[35]" : 0.0,
									"live.button[36]" : 0.0,
									"live.button[37]" : 0.0,
									"live.button[38]" : 0.0,
									"live.button[39]" : 0.0,
									"live.button[3]" : 0.0,
									"live.button[40]" : 0.0,
									"live.button[41]" : 0.0,
									"live.button[42]" : 0.0,
									"live.button[43]" : 0.0,
									"live.button[44]" : 0.0,
									"live.button[45]" : 0.0,
									"live.button[46]" : 0.0,
									"live.button[47]" : 0.0,
									"live.button[48]" : 0.0,
									"live.button[49]" : 0.0,
									"live.button[4]" : 0.0,
									"live.button[50]" : 0.0,
									"live.button[51]" : 0.0,
									"live.button[52]" : 0.0,
									"live.button[53]" : 0.0,
									"live.button[54]" : 0.0,
									"live.button[55]" : 0.0,
									"live.button[56]" : 0.0,
									"live.button[57]" : 0.0,
									"live.button[58]" : 0.0,
									"live.button[59]" : 0.0,
									"live.button[5]" : 0.0,
									"live.button[60]" : 0.0,
									"live.button[61]" : 0.0,
									"live.button[62]" : 0.0,
									"live.button[63]" : 0.0,
									"live.button[64]" : 0.0,
									"live.button[65]" : 0.0,
									"live.button[66]" : 0.0,
									"live.button[67]" : 0.0,
									"live.button[68]" : 0.0,
									"live.button[69]" : 0.0,
									"live.button[6]" : 0.0,
									"live.button[7]" : 0.0,
									"live.button[8]" : 0.0,
									"live.button[9]" : 0.0,
									"live.dial[10]" : 108.682055549781055,
									"live.dial[11]" : 6173.0,
									"live.dial[1]" : 70.14173228346354,
									"live.dial[3]" : 257.826207108760343,
									"live.dial[4]" : 6519.000000000000909,
									"live.dial[5]" : 6454.000000000001819,
									"live.dial[7]" : 135.233687156586029,
									"live.dial[8]" : 6917.0,
									"live.gain~" : -70.0,
									"live.menu" : 1.0,
									"live.menu[1]" : 0.0,
									"live.menu[2]" : 1.0,
									"live.menu[3]" : 3.0,
									"live.menu[4]" : 1.0,
									"live.menu[5]" : 2.0,
									"live.menu[6]" : 1.0,
									"live.menu[7]" : 1.0,
									"live.numbox" : 0.0,
									"live.numbox[1]" : 0.0,
									"live.numbox[2]" : 0.0,
									"live.numbox[3]" : 0.0,
									"live.text[10]" : 0.0,
									"live.text[11]" : 0.0,
									"live.text[12]" : 0.0,
									"live.text[13]" : 0.0,
									"live.text[14]" : 0.0,
									"live.text[15]" : 0.0,
									"live.text[16]" : 0.0,
									"live.text[17]" : 0.0,
									"live.text[18]" : 0.0,
									"live.text[19]" : 0.0,
									"live.text[1]" : 0.0,
									"live.text[20]" : 0.0,
									"live.text[24]" : 0.0,
									"live.text[28]" : 0.0,
									"live.text[29]" : 0.0,
									"live.text[2]" : 0.0,
									"live.text[30]" : 0.0,
									"live.text[31]" : 0.0,
									"live.text[32]" : 0.0,
									"live.text[33]" : 0.0,
									"live.text[34]" : 0.0,
									"live.text[3]" : 0.0,
									"live.text[4]" : 0.0,
									"live.text[5]" : 0.0,
									"live.text[6]" : 0.0,
									"live.text[70]" : 0.0,
									"live.text[71]" : 0.0,
									"live.text[78]" : 0.0,
									"live.text[79]" : 0.0,
									"live.text[7]" : 0.0,
									"live.text[8]" : 0.0,
									"live.text[9]" : 0.0,
									"mc.live.gain~[1]" : 0.0,
									"mc.live.gain~[3]" : 0.0,
									"mc.live.gain~[4]" : 0.0,
									"mc.live.gain~[5]" : 0.0,
									"pushtext16[1]" : 0.0,
									"pushtext18[1]" : 0.0,
									"pushtext20[1]" : 0.0,
									"pushtext22[1]" : 1.0,
									"pushtext2[1]" : 0.0,
									"pushtext3[1]" : 0.0,
									"pushtext4[1]" : 0.0,
									"pushtext5[1]" : 0.0,
									"blob" : 									{
										"flonum[1]" : [ 0.5 ],
										"flonum[2]" : [ 0.5 ],
										"flonum[3]" : [ 1.0 ],
										"flonum[4]" : [ 1.0 ],
										"flonum[5]" : [ 0.5 ],
										"flonum[6]" : [ 1.0 ],
										"flonum[7]" : [ 0.5 ],
										"flonum[8]" : [ 1.0 ],
										"live.grid" : [ 0, 16, 16, 0, 16, 0, 1001, 2002, 3003, 4004, 5005, 6006, 7007, 8008, 9009, 10010, 11011, 12012, 13013, 14014, 15015, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 ],
										"live.grid[1]" : [ 3, 16, 8, 0, 15, 7, 1004, 2006, 3004, 4003, 5002, 6007, 7004, 8007, 9004, 10006, 11001, 13004, 14001, 15005, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 ],
										"live.grid[2]" : [ 3, 16, 8, 0, 5, 2002, 3000, 8002, 12002, 15005, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 ],
										"live.grid[5]" : [ 3, 16, 8, 0, 6, 0, 3001, 7000, 10001, 13005, 14000, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 ],
										"live.grid[6]" : [ 3, 16, 8, 0, 7, 2005, 5006, 6004, 9001, 11004, 12005, 14001, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 ],
										"multislider" : [ 0.600477832363498, 1.0, 1.0, 1.0, 1.0, 1.0, 0.77570683879237, 0.911111071032862 ],
										"multislider[1]" : [ 1.0, 1.0, 1.0, 0.991756252319582, 1.0, 1.0, 1.0, 1.0 ],
										"multislider[2]" : [ 0.727917109766314, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 ],
										"multislider[3]" : [ 0.648267561389554, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 ],
										"number[10]" : [ 100 ],
										"number[11]" : [ 1000 ],
										"number[12]" : [ 1288 ],
										"number[13]" : [ 100 ],
										"number[14]" : [ 303 ],
										"number[15]" : [ 100 ],
										"number[16]" : [ 50000 ],
										"number[17]" : [ 1695 ],
										"number[18]" : [ 600 ],
										"number[19]" : [ 100 ],
										"number[20]" : [ 1000 ],
										"number[21]" : [ 2581 ],
										"number[22]" : [ 100 ],
										"number[23]" : [ 615 ],
										"number[24]" : [ 600 ],
										"number[25]" : [ 100 ],
										"number[26]" : [ 1000 ],
										"number[27]" : [ 602 ],
										"number[28]" : [ 100 ],
										"number[29]" : [ 1052 ],
										"number[6]" : [ 1000 ],
										"number[7]" : [ 100 ],
										"number[8]" : [ 600 ],
										"number[9]" : [ 600 ]
									}

								}

							}

						}
,
						"fileref" : 						{
							"name" : "cmmsd",
							"filename" : "may21[1].maxsnap",
							"filepath" : "~/Documents/Max 9/Snapshots",
							"filepos" : -1,
							"snapshotfileid" : "3f37358e88f9617384a98a288ebce5b2"
						}

					}
 ]
			}

		}
,
		"oscsendudpaddr" : "localhost"
	}

}
