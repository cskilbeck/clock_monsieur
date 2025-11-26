#pragma once

#include <cstdint>
#include <cstddef>

// Constants for the is_leaf field
#define NODE 0
#define LEAF 1

// Define the structure for a timezone node
struct tz_node_t
{
    uint16_t name_offset;
    uint16_t is_leaf : 1;
    uint16_t pad : 15;
    union
    {
        struct
        {
            // 'children_index' is an index into the TZ_NODES array
            int children_index;
            int num_children;
        };
        struct
        {
            float latitude;
            float longitude;
        };
    };
};

// The big string containing all timezone names, null-separated.
// Total size (including final null): 3724 bytes.
const char TZ_NAME_STRING[] = "Africa\0America\0Antarctica\0Arctic\0Asia\0Atlantic\0Australia\0Europe\0Indian\0"
                              "Pacific\0Abidjan\0Accra\0Addis Ababa\0Algiers\0Asmara\0Bamako\0Bangui\0Banjul\0"
                              "Bissau\0Blantyre\0Brazzaville\0Bujumbura\0Cairo\0Casablanca\0Ceuta\0Conakry\0"
                              "Dakar\0Dar es Salaam\0Djibouti\0Douala\0El Aaiun\0Freetown\0Gaborone\0Harare\0"
                              "Johannesburg\0Juba\0Kampala\0Khartoum\0Kigali\0Kinshasa\0Lagos\0Libreville\0"
                              "Lome\0Luanda\0Lubumbashi\0Lusaka\0Malabo\0Maputo\0Maseru\0Mbabane\0Mogadishu\0"
                              "Monrovia\0Nairobi\0Ndjamena\0Niamey\0Nouakchott\0Ouagadougou\0Porto-Novo\0"
                              "Sao Tome\0Tripoli\0Tunis\0Windhoek\0Adak\0Anchorage\0Anguilla\0Antigua\0"
                              "Araguaina\0Argentina\0Aruba\0Asuncion\0Atikokan\0Bahia\0Bahia Banderas\0"
                              "Barbados\0Belem\0Belize\0Blanc-Sablon\0Boa Vista\0Bogota\0Boise\0Cambridge Bay\0"
                              "Campo Grande\0Cancun\0Caracas\0Cayenne\0Cayman\0Chicago\0Chihuahua\0"
                              "Ciudad Juarez\0Costa Rica\0Coyhaique\0Creston\0Cuiaba\0Curacao\0Danmarkshavn\0"
                              "Dawson\0Dawson Creek\0Denver\0Detroit\0Dominica\0Edmonton\0Eirunepe\0"
                              "El Salvador\0Fort Nelson\0Fortaleza\0Glace Bay\0Goose Bay\0Grand Turk\0Grenada\0"
                              "Guadeloupe\0Guatemala\0Guayaquil\0Guyana\0Halifax\0Havana\0Hermosillo\0Indiana\0"
                              "Inuvik\0Iqaluit\0Jamaica\0Juneau\0Kentucky\0Kralendijk\0La Paz\0Lima\0"
                              "Los Angeles\0Lower Princes\0Maceio\0Managua\0Manaus\0Marigot\0Martinique\0"
                              "Matamoros\0Mazatlan\0Menominee\0Merida\0Metlakatla\0Mexico City\0Miquelon\0"
                              "Moncton\0Monterrey\0Montevideo\0Montserrat\0Nassau\0New York\0Nome\0Noronha\0"
                              "North Dakota\0Nuuk\0Ojinaga\0Panama\0Paramaribo\0Phoenix\0Port of Spain\0"
                              "Port-au-Prince\0Porto Velho\0Puerto Rico\0Punta Arenas\0Rankin Inlet\0Recife\0"
                              "Regina\0Resolute\0Rio Branco\0Santarem\0Santiago\0Santo Domingo\0Sao Paulo\0"
                              "Scoresbysund\0Sitka\0St Barthelemy\0St Johns\0St Kitts\0St Lucia\0St Thomas\0"
                              "St Vincent\0Swift Current\0Tegucigalpa\0Thule\0Tijuana\0Toronto\0Tortola\0"
                              "Vancouver\0Whitehorse\0Winnipeg\0Yakutat\0Casey\0Davis\0DumontDUrville\0"
                              "Macquarie\0Mawson\0McMurdo\0Palmer\0Rothera\0Syowa\0Troll\0Vostok\0"
                              "Longyearbyen\0Aden\0Almaty\0Amman\0Anadyr\0Aqtau\0Aqtobe\0Ashgabat\0Atyrau\0"
                              "Baghdad\0Bahrain\0Baku\0Bangkok\0Barnaul\0Beirut\0Bishkek\0Brunei\0Chita\0"
                              "Colombo\0Damascus\0Dhaka\0Dili\0Dubai\0Dushanbe\0Famagusta\0Gaza\0Hebron\0"
                              "Ho Chi Minh\0Hong Kong\0Hovd\0Irkutsk\0Jakarta\0Jayapura\0Jerusalem\0Kabul\0"
                              "Kamchatka\0Karachi\0Kathmandu\0Khandyga\0Kolkata\0Krasnoyarsk\0Kuala Lumpur\0"
                              "Kuching\0Kuwait\0Macau\0Magadan\0Makassar\0Manila\0Muscat\0Nicosia\0"
                              "Novokuznetsk\0Novosibirsk\0Omsk\0Oral\0Phnom Penh\0Pontianak\0Pyongyang\0Qatar\0"
                              "Qostanay\0Qyzylorda\0Riyadh\0Sakhalin\0Samarkand\0Seoul\0Shanghai\0Singapore\0"
                              "Srednekolymsk\0Taipei\0Tashkent\0Tbilisi\0Tehran\0Thimphu\0Tokyo\0Tomsk\0"
                              "Ulaanbaatar\0Urumqi\0Ust-Nera\0Vientiane\0Vladivostok\0Yakutsk\0Yangon\0"
                              "Yekaterinburg\0Yerevan\0Azores\0Bermuda\0Canary\0Cape Verde\0Faroe\0Madeira\0"
                              "Reykjavik\0South Georgia\0St Helena\0Stanley\0Adelaide\0Brisbane\0Broken Hill\0"
                              "Darwin\0Eucla\0Hobart\0Lindeman\0Lord Howe\0Melbourne\0Perth\0Sydney\0"
                              "Amsterdam\0Andorra\0Astrakhan\0Athens\0Belgrade\0Berlin\0Bratislava\0Brussels\0"
                              "Bucharest\0Budapest\0Busingen\0Chisinau\0Copenhagen\0Dublin\0Gibraltar\0"
                              "Guernsey\0Helsinki\0Isle of Man\0Istanbul\0Jersey\0Kaliningrad\0Kirov\0Kyiv\0"
                              "Lisbon\0Ljubljana\0London\0Luxembourg\0Madrid\0Malta\0Mariehamn\0Minsk\0Monaco\0"
                              "Moscow\0Oslo\0Paris\0Podgorica\0Prague\0Riga\0Rome\0Samara\0San Marino\0"
                              "Sarajevo\0Saratov\0Simferopol\0Skopje\0Sofia\0Stockholm\0Tallinn\0Tirane\0"
                              "Ulyanovsk\0Vaduz\0Vatican\0Vienna\0Vilnius\0Volgograd\0Warsaw\0Zagreb\0Zurich\0"
                              "Antananarivo\0Chagos\0Christmas\0Cocos\0Comoro\0Kerguelen\0Mahe\0Maldives\0"
                              "Mauritius\0Mayotte\0Reunion\0Apia\0Auckland\0Bougainville\0Chatham\0Chuuk\0"
                              "Easter\0Efate\0Fakaofo\0Fiji\0Funafuti\0Galapagos\0Gambier\0Guadalcanal\0Guam\0"
                              "Honolulu\0Kanton\0Kiritimati\0Kosrae\0Kwajalein\0Majuro\0Marquesas\0Midway\0"
                              "Nauru\0Niue\0Norfolk\0Noumea\0Pago Pago\0Palau\0Pitcairn\0Pohnpei\0"
                              "Port Moresby\0Rarotonga\0Saipan\0Tahiti\0Tarawa\0Tongatapu\0Wake\0Wallis\0"
                              "Buenos Aires\0Catamarca\0Cordoba\0Jujuy\0La Rioja\0Mendoza\0Rio Gallegos\0"
                              "Salta\0San Juan\0San Luis\0Tucuman\0Ushuaia\0Indianapolis\0Knox\0Marengo\0"
                              "Petersburg\0Tell City\0Vevay\0Vincennes\0Winamac\0Louisville\0Monticello\0"
                              "Beulah\0Center\0New Salem\0";

// Array of all timezone nodes. Total nodes: 432
const tz_node_t TZ_NODES[432] = {
    { 0, TZ_NODE, 0, { 10, 52 } },                        // Africa
    { 7, TZ_NODE, 0, { 62, 123 } },                       // America
    { 15, TZ_NODE, 0, { 185, 11 } },                      // Antarctica
    { 26, TZ_NODE, 0, { 196, 1 } },                       // Arctic
    { 33, TZ_NODE, 0, { 197, 82 } },                      // Asia
    { 38, TZ_NODE, 0, { 279, 10 } },                      // Atlantic
    { 47, TZ_NODE, 0, { 289, 11 } },                      // Australia
    { 57, TZ_NODE, 0, { 300, 58 } },                      // Europe
    { 64, TZ_NODE, 0, { 358, 11 } },                      // Indian
    { 71, TZ_NODE, 0, { 369, 38 } },                      // Pacific
    { 79, TZ_LEAF, 0, { 5.31667f, -4.03333f } },          // Abidjan
    { 87, TZ_LEAF, 0, { 5.55000f, -0.21667f } },          // Accra
    { 93, TZ_LEAF, 0, { 9.03333f, 38.70000f } },          // Addis Ababa
    { 105, TZ_LEAF, 0, { 36.78333f, 3.05000f } },         // Algiers
    { 113, TZ_LEAF, 0, { 15.33333f, 38.88333f } },        // Asmara
    { 120, TZ_LEAF, 0, { 12.65000f, -8.00000f } },        // Bamako
    { 127, TZ_LEAF, 0, { 4.36667f, 18.58333f } },         // Bangui
    { 134, TZ_LEAF, 0, { 13.46667f, -16.65000f } },       // Banjul
    { 141, TZ_LEAF, 0, { 11.85000f, -15.58333f } },       // Bissau
    { 148, TZ_LEAF, 0, { -15.78333f, 35.00000f } },       // Blantyre
    { 157, TZ_LEAF, 0, { -4.26667f, 15.28333f } },        // Brazzaville
    { 169, TZ_LEAF, 0, { -3.38333f, 29.36667f } },        // Bujumbura
    { 179, TZ_LEAF, 0, { 30.05000f, 31.25000f } },        // Cairo
    { 185, TZ_LEAF, 0, { 33.65000f, -7.58333f } },        // Casablanca
    { 196, TZ_LEAF, 0, { 35.88333f, -5.31667f } },        // Ceuta
    { 202, TZ_LEAF, 0, { 9.51667f, -13.71667f } },        // Conakry
    { 210, TZ_LEAF, 0, { 14.66667f, -17.43333f } },       // Dakar
    { 216, TZ_LEAF, 0, { -6.80000f, 39.28333f } },        // Dar es Salaam
    { 230, TZ_LEAF, 0, { 11.60000f, 43.15000f } },        // Djibouti
    { 239, TZ_LEAF, 0, { 4.05000f, 9.70000f } },          // Douala
    { 246, TZ_LEAF, 0, { 27.15000f, -13.20000f } },       // El Aaiun
    { 255, TZ_LEAF, 0, { 8.50000f, -13.25000f } },        // Freetown
    { 264, TZ_LEAF, 0, { -24.65000f, 25.91667f } },       // Gaborone
    { 273, TZ_LEAF, 0, { -17.83333f, 31.05000f } },       // Harare
    { 280, TZ_LEAF, 0, { -26.25000f, 28.00000f } },       // Johannesburg
    { 293, TZ_LEAF, 0, { 4.85000f, 31.61667f } },         // Juba
    { 298, TZ_LEAF, 0, { 0.31667f, 32.41667f } },         // Kampala
    { 306, TZ_LEAF, 0, { 15.60000f, 32.53333f } },        // Khartoum
    { 315, TZ_LEAF, 0, { -1.95000f, 30.06667f } },        // Kigali
    { 322, TZ_LEAF, 0, { -4.30000f, 15.30000f } },        // Kinshasa
    { 331, TZ_LEAF, 0, { 6.45000f, 3.40000f } },          // Lagos
    { 337, TZ_LEAF, 0, { 0.38333f, 9.45000f } },          // Libreville
    { 348, TZ_LEAF, 0, { 6.13333f, 1.21667f } },          // Lome
    { 353, TZ_LEAF, 0, { -8.80000f, 13.23333f } },        // Luanda
    { 360, TZ_LEAF, 0, { -11.66667f, 27.46667f } },       // Lubumbashi
    { 371, TZ_LEAF, 0, { -15.41667f, 28.28333f } },       // Lusaka
    { 378, TZ_LEAF, 0, { 3.75000f, 8.78333f } },          // Malabo
    { 385, TZ_LEAF, 0, { -25.96667f, 32.58333f } },       // Maputo
    { 392, TZ_LEAF, 0, { -29.46667f, 27.50000f } },       // Maseru
    { 399, TZ_LEAF, 0, { -26.30000f, 31.10000f } },       // Mbabane
    { 407, TZ_LEAF, 0, { 2.06667f, 45.36667f } },         // Mogadishu
    { 417, TZ_LEAF, 0, { 6.30000f, -10.78333f } },        // Monrovia
    { 426, TZ_LEAF, 0, { -1.28333f, 36.81667f } },        // Nairobi
    { 434, TZ_LEAF, 0, { 12.11667f, 15.05000f } },        // Ndjamena
    { 443, TZ_LEAF, 0, { 13.51667f, 2.11667f } },         // Niamey
    { 450, TZ_LEAF, 0, { 18.10000f, -15.95000f } },       // Nouakchott
    { 461, TZ_LEAF, 0, { 12.36667f, -1.51667f } },        // Ouagadougou
    { 473, TZ_LEAF, 0, { 6.48333f, 2.61667f } },          // Porto-Novo
    { 484, TZ_LEAF, 0, { 0.33333f, 6.73333f } },          // Sao Tome
    { 493, TZ_LEAF, 0, { 32.90000f, 13.18333f } },        // Tripoli
    { 501, TZ_LEAF, 0, { 36.80000f, 10.18333f } },        // Tunis
    { 507, TZ_LEAF, 0, { -22.56667f, 17.10000f } },       // Windhoek
    { 516, TZ_LEAF, 0, { 51.88000f, -176.65806f } },      // Adak
    { 521, TZ_LEAF, 0, { 61.21806f, -149.90028f } },      // Anchorage
    { 531, TZ_LEAF, 0, { 18.20000f, -63.06667f } },       // Anguilla
    { 540, TZ_LEAF, 0, { 17.05000f, -61.80000f } },       // Antigua
    { 548, TZ_LEAF, 0, { -7.20000f, -48.20000f } },       // Araguaina
    { 558, TZ_NODE, 0, { 407, 12 } },                     // Argentina
    { 568, TZ_LEAF, 0, { 12.50000f, -69.96667f } },       // Aruba
    { 574, TZ_LEAF, 0, { -25.26667f, -57.66667f } },      // Asuncion
    { 583, TZ_LEAF, 0, { 48.75861f, -91.62167f } },       // Atikokan
    { 592, TZ_LEAF, 0, { -12.98333f, -38.51667f } },      // Bahia
    { 598, TZ_LEAF, 0, { 20.80000f, -105.25000f } },      // Bahia Banderas
    { 613, TZ_LEAF, 0, { 13.10000f, -59.61667f } },       // Barbados
    { 622, TZ_LEAF, 0, { -1.45000f, -48.48333f } },       // Belem
    { 628, TZ_LEAF, 0, { 17.50000f, -88.20000f } },       // Belize
    { 635, TZ_LEAF, 0, { 51.41667f, -57.11667f } },       // Blanc-Sablon
    { 648, TZ_LEAF, 0, { 2.81667f, -60.66667f } },        // Boa Vista
    { 658, TZ_LEAF, 0, { 4.60000f, -74.08333f } },        // Bogota
    { 665, TZ_LEAF, 0, { 43.61361f, -116.20250f } },      // Boise
    { 671, TZ_LEAF, 0, { 69.11389f, -105.05278f } },      // Cambridge Bay
    { 685, TZ_LEAF, 0, { -20.45000f, -54.61667f } },      // Campo Grande
    { 698, TZ_LEAF, 0, { 21.08333f, -86.76667f } },       // Cancun
    { 705, TZ_LEAF, 0, { 10.50000f, -66.93333f } },       // Caracas
    { 713, TZ_LEAF, 0, { 4.93333f, -52.33333f } },        // Cayenne
    { 721, TZ_LEAF, 0, { 19.30000f, -81.38333f } },       // Cayman
    { 728, TZ_LEAF, 0, { 41.85000f, -87.65000f } },       // Chicago
    { 736, TZ_LEAF, 0, { 28.63333f, -106.08333f } },      // Chihuahua
    { 746, TZ_LEAF, 0, { 31.73333f, -106.48333f } },      // Ciudad Juarez
    { 760, TZ_LEAF, 0, { 9.93333f, -84.08333f } },        // Costa Rica
    { 771, TZ_LEAF, 0, { -45.56667f, -72.06667f } },      // Coyhaique
    { 781, TZ_LEAF, 0, { 49.10000f, -116.51667f } },      // Creston
    { 789, TZ_LEAF, 0, { -15.58333f, -56.08333f } },      // Cuiaba
    { 796, TZ_LEAF, 0, { 12.18333f, -69.00000f } },       // Curacao
    { 804, TZ_LEAF, 0, { 76.76667f, -18.66667f } },       // Danmarkshavn
    { 817, TZ_LEAF, 0, { 64.06667f, -139.41667f } },      // Dawson
    { 824, TZ_LEAF, 0, { 55.76667f, -120.23333f } },      // Dawson Creek
    { 837, TZ_LEAF, 0, { 39.73917f, -104.98417f } },      // Denver
    { 844, TZ_LEAF, 0, { 42.33139f, -83.04583f } },       // Detroit
    { 852, TZ_LEAF, 0, { 15.30000f, -61.40000f } },       // Dominica
    { 861, TZ_LEAF, 0, { 53.55000f, -113.46667f } },      // Edmonton
    { 870, TZ_LEAF, 0, { -6.66667f, -69.86667f } },       // Eirunepe
    { 879, TZ_LEAF, 0, { 13.70000f, -89.20000f } },       // El Salvador
    { 891, TZ_LEAF, 0, { 58.80000f, -122.70000f } },      // Fort Nelson
    { 903, TZ_LEAF, 0, { -3.71667f, -38.50000f } },       // Fortaleza
    { 913, TZ_LEAF, 0, { 46.20000f, -59.95000f } },       // Glace Bay
    { 923, TZ_LEAF, 0, { 53.33333f, -60.41667f } },       // Goose Bay
    { 933, TZ_LEAF, 0, { 21.46667f, -71.13333f } },       // Grand Turk
    { 944, TZ_LEAF, 0, { 12.05000f, -61.75000f } },       // Grenada
    { 952, TZ_LEAF, 0, { 16.23333f, -61.53333f } },       // Guadeloupe
    { 963, TZ_LEAF, 0, { 14.63333f, -90.51667f } },       // Guatemala
    { 973, TZ_LEAF, 0, { -2.16667f, -79.83333f } },       // Guayaquil
    { 983, TZ_LEAF, 0, { 6.80000f, -58.16667f } },        // Guyana
    { 990, TZ_LEAF, 0, { 44.65000f, -63.60000f } },       // Halifax
    { 998, TZ_LEAF, 0, { 23.13333f, -82.36667f } },       // Havana
    { 1005, TZ_LEAF, 0, { 29.06667f, -110.96667f } },     // Hermosillo
    { 1016, TZ_NODE, 0, { 419, 8 } },                     // Indiana
    { 1024, TZ_LEAF, 0, { 68.34972f, -133.71667f } },     // Inuvik
    { 1031, TZ_LEAF, 0, { 63.73333f, -68.46667f } },      // Iqaluit
    { 1039, TZ_LEAF, 0, { 17.96806f, -76.79333f } },      // Jamaica
    { 1047, TZ_LEAF, 0, { 58.30194f, -134.41972f } },     // Juneau
    { 1054, TZ_NODE, 0, { 427, 2 } },                     // Kentucky
    { 1063, TZ_LEAF, 0, { 12.15083f, -68.27667f } },      // Kralendijk
    { 1074, TZ_LEAF, 0, { -16.50000f, -68.15000f } },     // La Paz
    { 1081, TZ_LEAF, 0, { -12.05000f, -77.05000f } },     // Lima
    { 1086, TZ_LEAF, 0, { 34.05222f, -118.24278f } },     // Los Angeles
    { 1098, TZ_LEAF, 0, { 18.05139f, -63.04722f } },      // Lower Princes
    { 1112, TZ_LEAF, 0, { -9.66667f, -35.71667f } },      // Maceio
    { 1119, TZ_LEAF, 0, { 12.15000f, -86.28333f } },      // Managua
    { 1127, TZ_LEAF, 0, { -3.13333f, -60.01667f } },      // Manaus
    { 1134, TZ_LEAF, 0, { 18.06667f, -63.08333f } },      // Marigot
    { 1142, TZ_LEAF, 0, { 14.60000f, -61.08333f } },      // Martinique
    { 1153, TZ_LEAF, 0, { 25.83333f, -97.50000f } },      // Matamoros
    { 1163, TZ_LEAF, 0, { 23.21667f, -106.41667f } },     // Mazatlan
    { 1172, TZ_LEAF, 0, { 45.10778f, -87.61417f } },      // Menominee
    { 1182, TZ_LEAF, 0, { 20.96667f, -89.61667f } },      // Merida
    { 1189, TZ_LEAF, 0, { 55.12694f, -131.57639f } },     // Metlakatla
    { 1200, TZ_LEAF, 0, { 19.40000f, -99.15000f } },      // Mexico City
    { 1212, TZ_LEAF, 0, { 47.05000f, -56.33333f } },      // Miquelon
    { 1221, TZ_LEAF, 0, { 46.10000f, -64.78333f } },      // Moncton
    { 1229, TZ_LEAF, 0, { 25.66667f, -100.31667f } },     // Monterrey
    { 1239, TZ_LEAF, 0, { -34.90917f, -56.21250f } },     // Montevideo
    { 1250, TZ_LEAF, 0, { 16.71667f, -62.21667f } },      // Montserrat
    { 1261, TZ_LEAF, 0, { 25.08333f, -77.35000f } },      // Nassau
    { 1268, TZ_LEAF, 0, { 40.71417f, -74.00639f } },      // New York
    { 1277, TZ_LEAF, 0, { 64.50111f, -165.40639f } },     // Nome
    { 1282, TZ_LEAF, 0, { -3.85000f, -32.41667f } },      // Noronha
    { 1290, TZ_NODE, 0, { 429, 3 } },                     // North Dakota
    { 1303, TZ_LEAF, 0, { 64.18333f, -51.73333f } },      // Nuuk
    { 1308, TZ_LEAF, 0, { 29.56667f, -104.41667f } },     // Ojinaga
    { 1316, TZ_LEAF, 0, { 8.96667f, -79.53333f } },       // Panama
    { 1323, TZ_LEAF, 0, { 5.83333f, -55.16667f } },       // Paramaribo
    { 1334, TZ_LEAF, 0, { 33.44833f, -112.07333f } },     // Phoenix
    { 1342, TZ_LEAF, 0, { 10.65000f, -61.51667f } },      // Port of Spain
    { 1356, TZ_LEAF, 0, { 18.53333f, -72.33333f } },      // Port-au-Prince
    { 1371, TZ_LEAF, 0, { -8.76667f, -63.90000f } },      // Porto Velho
    { 1383, TZ_LEAF, 0, { 18.46833f, -66.10611f } },      // Puerto Rico
    { 1395, TZ_LEAF, 0, { -53.15000f, -70.91667f } },     // Punta Arenas
    { 1408, TZ_LEAF, 0, { 62.81667f, -92.08306f } },      // Rankin Inlet
    { 1421, TZ_LEAF, 0, { -8.05000f, -34.90000f } },      // Recife
    { 1428, TZ_LEAF, 0, { 50.40000f, -104.65000f } },     // Regina
    { 1435, TZ_LEAF, 0, { 74.69556f, -94.82917f } },      // Resolute
    { 1444, TZ_LEAF, 0, { -9.96667f, -67.80000f } },      // Rio Branco
    { 1455, TZ_LEAF, 0, { -2.43333f, -54.86667f } },      // Santarem
    { 1464, TZ_LEAF, 0, { -33.45000f, -70.66667f } },     // Santiago
    { 1473, TZ_LEAF, 0, { 18.46667f, -69.90000f } },      // Santo Domingo
    { 1487, TZ_LEAF, 0, { -23.53333f, -46.61667f } },     // Sao Paulo
    { 1497, TZ_LEAF, 0, { 70.48333f, -21.96667f } },      // Scoresbysund
    { 1510, TZ_LEAF, 0, { 57.17639f, -135.30194f } },     // Sitka
    { 1516, TZ_LEAF, 0, { 17.88333f, -62.85000f } },      // St Barthelemy
    { 1530, TZ_LEAF, 0, { 47.56667f, -52.71667f } },      // St Johns
    { 1539, TZ_LEAF, 0, { 17.30000f, -62.71667f } },      // St Kitts
    { 1548, TZ_LEAF, 0, { 14.01667f, -61.00000f } },      // St Lucia
    { 1557, TZ_LEAF, 0, { 18.35000f, -64.93333f } },      // St Thomas
    { 1567, TZ_LEAF, 0, { 13.15000f, -61.23333f } },      // St Vincent
    { 1578, TZ_LEAF, 0, { 50.28333f, -107.83333f } },     // Swift Current
    { 1592, TZ_LEAF, 0, { 14.10000f, -87.21667f } },      // Tegucigalpa
    { 1604, TZ_LEAF, 0, { 76.56667f, -68.78333f } },      // Thule
    { 1610, TZ_LEAF, 0, { 32.53333f, -117.01667f } },     // Tijuana
    { 1618, TZ_LEAF, 0, { 43.65000f, -79.38333f } },      // Toronto
    { 1626, TZ_LEAF, 0, { 18.45000f, -64.61667f } },      // Tortola
    { 1634, TZ_LEAF, 0, { 49.26667f, -123.11667f } },     // Vancouver
    { 1644, TZ_LEAF, 0, { 60.71667f, -135.05000f } },     // Whitehorse
    { 1655, TZ_LEAF, 0, { 49.88333f, -97.15000f } },      // Winnipeg
    { 1664, TZ_LEAF, 0, { 59.54694f, -139.72722f } },     // Yakutat
    { 1672, TZ_LEAF, 0, { -66.28333f, 110.51667f } },     // Casey
    { 1678, TZ_LEAF, 0, { -68.58333f, 77.96667f } },      // Davis
    { 1684, TZ_LEAF, 0, { -66.66667f, 140.01667f } },     // DumontDUrville
    { 1699, TZ_LEAF, 0, { -54.50000f, 158.95000f } },     // Macquarie
    { 1709, TZ_LEAF, 0, { -67.60000f, 62.88333f } },      // Mawson
    { 1716, TZ_LEAF, 0, { -77.83333f, 166.60000f } },     // McMurdo
    { 1724, TZ_LEAF, 0, { -64.80000f, -64.10000f } },     // Palmer
    { 1731, TZ_LEAF, 0, { -67.56667f, -68.13333f } },     // Rothera
    { 1739, TZ_LEAF, 0, { -69.00611f, 39.59000f } },      // Syowa
    { 1745, TZ_LEAF, 0, { -72.01139f, 2.53500f } },       // Troll
    { 1751, TZ_LEAF, 0, { -78.40000f, 106.90000f } },     // Vostok
    { 1758, TZ_LEAF, 0, { 78.00000f, 16.00000f } },       // Longyearbyen
    { 1771, TZ_LEAF, 0, { 12.75000f, 45.20000f } },       // Aden
    { 1776, TZ_LEAF, 0, { 43.25000f, 76.95000f } },       // Almaty
    { 1783, TZ_LEAF, 0, { 31.95000f, 35.93333f } },       // Amman
    { 1789, TZ_LEAF, 0, { 64.75000f, 177.48333f } },      // Anadyr
    { 1796, TZ_LEAF, 0, { 44.51667f, 50.26667f } },       // Aqtau
    { 1802, TZ_LEAF, 0, { 50.28333f, 57.16667f } },       // Aqtobe
    { 1809, TZ_LEAF, 0, { 37.95000f, 58.38333f } },       // Ashgabat
    { 1818, TZ_LEAF, 0, { 47.11667f, 51.93333f } },       // Atyrau
    { 1825, TZ_LEAF, 0, { 33.35000f, 44.41667f } },       // Baghdad
    { 1833, TZ_LEAF, 0, { 26.38333f, 50.58333f } },       // Bahrain
    { 1841, TZ_LEAF, 0, { 40.38333f, 49.85000f } },       // Baku
    { 1846, TZ_LEAF, 0, { 13.75000f, 100.51667f } },      // Bangkok
    { 1854, TZ_LEAF, 0, { 53.36667f, 83.75000f } },       // Barnaul
    { 1862, TZ_LEAF, 0, { 33.88333f, 35.50000f } },       // Beirut
    { 1869, TZ_LEAF, 0, { 42.90000f, 74.60000f } },       // Bishkek
    { 1877, TZ_LEAF, 0, { 4.93333f, 114.91667f } },       // Brunei
    { 1884, TZ_LEAF, 0, { 52.05000f, 113.46667f } },      // Chita
    { 1890, TZ_LEAF, 0, { 6.93333f, 79.85000f } },        // Colombo
    { 1898, TZ_LEAF, 0, { 33.50000f, 36.30000f } },       // Damascus
    { 1907, TZ_LEAF, 0, { 23.71667f, 90.41667f } },       // Dhaka
    { 1913, TZ_LEAF, 0, { -8.55000f, 125.58333f } },      // Dili
    { 1918, TZ_LEAF, 0, { 25.30000f, 55.30000f } },       // Dubai
    { 1924, TZ_LEAF, 0, { 38.58333f, 68.80000f } },       // Dushanbe
    { 1933, TZ_LEAF, 0, { 35.11667f, 33.95000f } },       // Famagusta
    { 1943, TZ_LEAF, 0, { 31.50000f, 34.46667f } },       // Gaza
    { 1948, TZ_LEAF, 0, { 31.53333f, 35.09500f } },       // Hebron
    { 1955, TZ_LEAF, 0, { 10.75000f, 106.66667f } },      // Ho Chi Minh
    { 1967, TZ_LEAF, 0, { 22.28333f, 114.15000f } },      // Hong Kong
    { 1977, TZ_LEAF, 0, { 48.01667f, 91.65000f } },       // Hovd
    { 1982, TZ_LEAF, 0, { 52.26667f, 104.33333f } },      // Irkutsk
    { 1990, TZ_LEAF, 0, { -6.16667f, 106.80000f } },      // Jakarta
    { 1998, TZ_LEAF, 0, { -2.53333f, 140.70000f } },      // Jayapura
    { 2007, TZ_LEAF, 0, { 31.78056f, 35.22389f } },       // Jerusalem
    { 2017, TZ_LEAF, 0, { 34.51667f, 69.20000f } },       // Kabul
    { 2023, TZ_LEAF, 0, { 53.01667f, 158.65000f } },      // Kamchatka
    { 2033, TZ_LEAF, 0, { 24.86667f, 67.05000f } },       // Karachi
    { 2041, TZ_LEAF, 0, { 27.71667f, 85.31667f } },       // Kathmandu
    { 2051, TZ_LEAF, 0, { 62.65639f, 135.55389f } },      // Khandyga
    { 2060, TZ_LEAF, 0, { 22.53333f, 88.36667f } },       // Kolkata
    { 2068, TZ_LEAF, 0, { 56.01667f, 92.83333f } },       // Krasnoyarsk
    { 2080, TZ_LEAF, 0, { 3.16667f, 101.70000f } },       // Kuala Lumpur
    { 2093, TZ_LEAF, 0, { 1.55000f, 110.33333f } },       // Kuching
    { 2101, TZ_LEAF, 0, { 29.33333f, 47.98333f } },       // Kuwait
    { 2108, TZ_LEAF, 0, { 22.19722f, 113.54167f } },      // Macau
    { 2114, TZ_LEAF, 0, { 59.56667f, 150.80000f } },      // Magadan
    { 2122, TZ_LEAF, 0, { -5.11667f, 119.40000f } },      // Makassar
    { 2131, TZ_LEAF, 0, { 14.58667f, 120.96778f } },      // Manila
    { 2138, TZ_LEAF, 0, { 23.60000f, 58.58333f } },       // Muscat
    { 2145, TZ_LEAF, 0, { 35.16667f, 33.36667f } },       // Nicosia
    { 2153, TZ_LEAF, 0, { 53.75000f, 87.11667f } },       // Novokuznetsk
    { 2166, TZ_LEAF, 0, { 55.03333f, 82.91667f } },       // Novosibirsk
    { 2178, TZ_LEAF, 0, { 55.00000f, 73.40000f } },       // Omsk
    { 2183, TZ_LEAF, 0, { 51.21667f, 51.35000f } },       // Oral
    { 2188, TZ_LEAF, 0, { 11.55000f, 104.91667f } },      // Phnom Penh
    { 2199, TZ_LEAF, 0, { -0.03333f, 109.33333f } },      // Pontianak
    { 2209, TZ_LEAF, 0, { 39.01667f, 125.75000f } },      // Pyongyang
    { 2219, TZ_LEAF, 0, { 25.28333f, 51.53333f } },       // Qatar
    { 2225, TZ_LEAF, 0, { 53.20000f, 63.61667f } },       // Qostanay
    { 2234, TZ_LEAF, 0, { 44.80000f, 65.46667f } },       // Qyzylorda
    { 2244, TZ_LEAF, 0, { 24.63333f, 46.71667f } },       // Riyadh
    { 2251, TZ_LEAF, 0, { 46.96667f, 142.70000f } },      // Sakhalin
    { 2260, TZ_LEAF, 0, { 39.66667f, 66.80000f } },       // Samarkand
    { 2270, TZ_LEAF, 0, { 37.55000f, 126.96667f } },      // Seoul
    { 2276, TZ_LEAF, 0, { 31.23333f, 121.46667f } },      // Shanghai
    { 2285, TZ_LEAF, 0, { 1.28333f, 103.85000f } },       // Singapore
    { 2295, TZ_LEAF, 0, { 67.46667f, 153.71667f } },      // Srednekolymsk
    { 2309, TZ_LEAF, 0, { 25.05000f, 121.50000f } },      // Taipei
    { 2316, TZ_LEAF, 0, { 41.33333f, 69.30000f } },       // Tashkent
    { 2325, TZ_LEAF, 0, { 41.71667f, 44.81667f } },       // Tbilisi
    { 2333, TZ_LEAF, 0, { 35.66667f, 51.43333f } },       // Tehran
    { 2340, TZ_LEAF, 0, { 27.46667f, 89.65000f } },       // Thimphu
    { 2348, TZ_LEAF, 0, { 35.65444f, 139.74472f } },      // Tokyo
    { 2354, TZ_LEAF, 0, { 56.50000f, 84.96667f } },       // Tomsk
    { 2360, TZ_LEAF, 0, { 47.91667f, 106.88333f } },      // Ulaanbaatar
    { 2372, TZ_LEAF, 0, { 43.80000f, 87.58333f } },       // Urumqi
    { 2379, TZ_LEAF, 0, { 64.56028f, 143.22667f } },      // Ust-Nera
    { 2388, TZ_LEAF, 0, { 17.96667f, 102.60000f } },      // Vientiane
    { 2398, TZ_LEAF, 0, { 43.16667f, 131.93333f } },      // Vladivostok
    { 2410, TZ_LEAF, 0, { 62.00000f, 129.66667f } },      // Yakutsk
    { 2418, TZ_LEAF, 0, { 16.78333f, 96.16667f } },       // Yangon
    { 2425, TZ_LEAF, 0, { 56.85000f, 60.60000f } },       // Yekaterinburg
    { 2439, TZ_LEAF, 0, { 40.18333f, 44.50000f } },       // Yerevan
    { 2447, TZ_LEAF, 0, { 37.73333f, -25.66667f } },      // Azores
    { 2454, TZ_LEAF, 0, { 32.28333f, -64.76667f } },      // Bermuda
    { 2462, TZ_LEAF, 0, { 28.10000f, -15.40000f } },      // Canary
    { 2469, TZ_LEAF, 0, { 14.91667f, -23.51667f } },      // Cape Verde
    { 2480, TZ_LEAF, 0, { 62.01667f, -6.76667f } },       // Faroe
    { 2486, TZ_LEAF, 0, { 32.63333f, -16.90000f } },      // Madeira
    { 2494, TZ_LEAF, 0, { 64.15000f, -21.85000f } },      // Reykjavik
    { 2504, TZ_LEAF, 0, { -54.26667f, -36.53333f } },     // South Georgia
    { 2518, TZ_LEAF, 0, { -15.91667f, -5.70000f } },      // St Helena
    { 2528, TZ_LEAF, 0, { -51.70000f, -57.85000f } },     // Stanley
    { 2536, TZ_LEAF, 0, { -34.91667f, 138.58333f } },     // Adelaide
    { 2545, TZ_LEAF, 0, { -27.46667f, 153.03333f } },     // Brisbane
    { 2554, TZ_LEAF, 0, { -31.95000f, 141.45000f } },     // Broken Hill
    { 2566, TZ_LEAF, 0, { -12.46667f, 130.83333f } },     // Darwin
    { 2573, TZ_LEAF, 0, { -31.71667f, 128.86667f } },     // Eucla
    { 2579, TZ_LEAF, 0, { -42.88333f, 147.31667f } },     // Hobart
    { 2586, TZ_LEAF, 0, { -20.26667f, 149.00000f } },     // Lindeman
    { 2595, TZ_LEAF, 0, { -31.55000f, 159.08333f } },     // Lord Howe
    { 2605, TZ_LEAF, 0, { -37.81667f, 144.96667f } },     // Melbourne
    { 2615, TZ_LEAF, 0, { -31.95000f, 115.85000f } },     // Perth
    { 2621, TZ_LEAF, 0, { -33.86667f, 151.21667f } },     // Sydney
    { 2628, TZ_LEAF, 0, { 52.36667f, 4.90000f } },        // Amsterdam
    { 2638, TZ_LEAF, 0, { 42.50000f, 1.51667f } },        // Andorra
    { 2646, TZ_LEAF, 0, { 46.35000f, 48.05000f } },       // Astrakhan
    { 2656, TZ_LEAF, 0, { 37.96667f, 23.71667f } },       // Athens
    { 2663, TZ_LEAF, 0, { 44.83333f, 20.50000f } },       // Belgrade
    { 2672, TZ_LEAF, 0, { 52.50000f, 13.36667f } },       // Berlin
    { 2679, TZ_LEAF, 0, { 48.15000f, 17.11667f } },       // Bratislava
    { 2690, TZ_LEAF, 0, { 50.83333f, 4.33333f } },        // Brussels
    { 2699, TZ_LEAF, 0, { 44.43333f, 26.10000f } },       // Bucharest
    { 2709, TZ_LEAF, 0, { 47.50000f, 19.08333f } },       // Budapest
    { 2718, TZ_LEAF, 0, { 47.70000f, 8.68333f } },        // Busingen
    { 2727, TZ_LEAF, 0, { 47.00000f, 28.83333f } },       // Chisinau
    { 2736, TZ_LEAF, 0, { 55.66667f, 12.58333f } },       // Copenhagen
    { 2747, TZ_LEAF, 0, { 53.33333f, -6.25000f } },       // Dublin
    { 2754, TZ_LEAF, 0, { 36.13333f, -5.35000f } },       // Gibraltar
    { 2764, TZ_LEAF, 0, { 49.45472f, -2.53611f } },       // Guernsey
    { 2773, TZ_LEAF, 0, { 60.16667f, 24.96667f } },       // Helsinki
    { 2782, TZ_LEAF, 0, { 54.15000f, -4.46667f } },       // Isle of Man
    { 2794, TZ_LEAF, 0, { 41.01667f, 28.96667f } },       // Istanbul
    { 2803, TZ_LEAF, 0, { 49.18361f, -2.10667f } },       // Jersey
    { 2810, TZ_LEAF, 0, { 54.71667f, 20.50000f } },       // Kaliningrad
    { 2822, TZ_LEAF, 0, { 58.60000f, 49.65000f } },       // Kirov
    { 2828, TZ_LEAF, 0, { 50.43333f, 30.51667f } },       // Kyiv
    { 2833, TZ_LEAF, 0, { 38.71667f, -9.13333f } },       // Lisbon
    { 2840, TZ_LEAF, 0, { 46.05000f, 14.51667f } },       // Ljubljana
    { 2850, TZ_LEAF, 0, { 51.50833f, -0.12528f } },       // London
    { 2857, TZ_LEAF, 0, { 49.60000f, 6.15000f } },        // Luxembourg
    { 2868, TZ_LEAF, 0, { 40.40000f, -3.68333f } },       // Madrid
    { 2875, TZ_LEAF, 0, { 35.90000f, 14.51667f } },       // Malta
    { 2881, TZ_LEAF, 0, { 60.10000f, 19.95000f } },       // Mariehamn
    { 2891, TZ_LEAF, 0, { 53.90000f, 27.56667f } },       // Minsk
    { 2897, TZ_LEAF, 0, { 43.70000f, 7.38333f } },        // Monaco
    { 2904, TZ_LEAF, 0, { 55.75583f, 37.61778f } },       // Moscow
    { 2911, TZ_LEAF, 0, { 59.91667f, 10.75000f } },       // Oslo
    { 2916, TZ_LEAF, 0, { 48.86667f, 2.33333f } },        // Paris
    { 2922, TZ_LEAF, 0, { 42.43333f, 19.26667f } },       // Podgorica
    { 2932, TZ_LEAF, 0, { 50.08333f, 14.43333f } },       // Prague
    { 2939, TZ_LEAF, 0, { 56.95000f, 24.10000f } },       // Riga
    { 2944, TZ_LEAF, 0, { 41.90000f, 12.48333f } },       // Rome
    { 2949, TZ_LEAF, 0, { 53.20000f, 50.15000f } },       // Samara
    { 2956, TZ_LEAF, 0, { 43.91667f, 12.46667f } },       // San Marino
    { 2967, TZ_LEAF, 0, { 43.86667f, 18.41667f } },       // Sarajevo
    { 2976, TZ_LEAF, 0, { 51.56667f, 46.03333f } },       // Saratov
    { 2984, TZ_LEAF, 0, { 44.95000f, 34.10000f } },       // Simferopol
    { 2995, TZ_LEAF, 0, { 41.98333f, 21.43333f } },       // Skopje
    { 3002, TZ_LEAF, 0, { 42.68333f, 23.31667f } },       // Sofia
    { 3008, TZ_LEAF, 0, { 59.33333f, 18.05000f } },       // Stockholm
    { 3018, TZ_LEAF, 0, { 59.41667f, 24.75000f } },       // Tallinn
    { 3026, TZ_LEAF, 0, { 41.33333f, 19.83333f } },       // Tirane
    { 3033, TZ_LEAF, 0, { 54.33333f, 48.40000f } },       // Ulyanovsk
    { 3043, TZ_LEAF, 0, { 47.15000f, 9.51667f } },        // Vaduz
    { 3049, TZ_LEAF, 0, { 41.90222f, 12.45306f } },       // Vatican
    { 3057, TZ_LEAF, 0, { 48.21667f, 16.33333f } },       // Vienna
    { 3064, TZ_LEAF, 0, { 54.68333f, 25.31667f } },       // Vilnius
    { 3072, TZ_LEAF, 0, { 48.73333f, 44.41667f } },       // Volgograd
    { 3082, TZ_LEAF, 0, { 52.25000f, 21.00000f } },       // Warsaw
    { 3089, TZ_LEAF, 0, { 45.80000f, 15.96667f } },       // Zagreb
    { 3096, TZ_LEAF, 0, { 47.38333f, 8.53333f } },        // Zurich
    { 3103, TZ_LEAF, 0, { -18.91667f, 47.51667f } },      // Antananarivo
    { 3116, TZ_LEAF, 0, { -7.33333f, 72.41667f } },       // Chagos
    { 3123, TZ_LEAF, 0, { -10.41667f, 105.71667f } },     // Christmas
    { 3133, TZ_LEAF, 0, { -12.16667f, 96.91667f } },      // Cocos
    { 3139, TZ_LEAF, 0, { -11.68333f, 43.26667f } },      // Comoro
    { 3146, TZ_LEAF, 0, { -49.35278f, 70.21750f } },      // Kerguelen
    { 3156, TZ_LEAF, 0, { -4.66667f, 55.46667f } },       // Mahe
    { 3161, TZ_LEAF, 0, { 4.16667f, 73.50000f } },        // Maldives
    { 3170, TZ_LEAF, 0, { -20.16667f, 57.50000f } },      // Mauritius
    { 3180, TZ_LEAF, 0, { -12.78333f, 45.23333f } },      // Mayotte
    { 3188, TZ_LEAF, 0, { -20.86667f, 55.46667f } },      // Reunion
    { 3196, TZ_LEAF, 0, { -13.83333f, -171.73333f } },    // Apia
    { 3201, TZ_LEAF, 0, { -36.86667f, 174.76667f } },     // Auckland
    { 3210, TZ_LEAF, 0, { -6.21667f, 155.56667f } },      // Bougainville
    { 3223, TZ_LEAF, 0, { -43.95000f, -176.55000f } },    // Chatham
    { 3231, TZ_LEAF, 0, { 7.41667f, 151.78333f } },       // Chuuk
    { 3237, TZ_LEAF, 0, { -27.15000f, -109.43333f } },    // Easter
    { 3244, TZ_LEAF, 0, { -17.66667f, 168.41667f } },     // Efate
    { 3250, TZ_LEAF, 0, { -9.36667f, -171.23333f } },     // Fakaofo
    { 3258, TZ_LEAF, 0, { -18.13333f, 178.41667f } },     // Fiji
    { 3263, TZ_LEAF, 0, { -8.51667f, 179.21667f } },      // Funafuti
    { 3272, TZ_LEAF, 0, { -0.90000f, -89.60000f } },      // Galapagos
    { 3282, TZ_LEAF, 0, { -23.13333f, -134.95000f } },    // Gambier
    { 3290, TZ_LEAF, 0, { -9.53333f, 160.20000f } },      // Guadalcanal
    { 3302, TZ_LEAF, 0, { 13.46667f, 144.75000f } },      // Guam
    { 3307, TZ_LEAF, 0, { 21.30694f, -157.85833f } },     // Honolulu
    { 3316, TZ_LEAF, 0, { -2.78333f, -171.71667f } },     // Kanton
    { 3323, TZ_LEAF, 0, { 1.86667f, -157.33333f } },      // Kiritimati
    { 3334, TZ_LEAF, 0, { 5.31667f, 162.98333f } },       // Kosrae
    { 3341, TZ_LEAF, 0, { 9.08333f, 167.33333f } },       // Kwajalein
    { 3351, TZ_LEAF, 0, { 7.15000f, 171.20000f } },       // Majuro
    { 3358, TZ_LEAF, 0, { -9.00000f, -139.50000f } },     // Marquesas
    { 3368, TZ_LEAF, 0, { 28.21667f, -177.36667f } },     // Midway
    { 3375, TZ_LEAF, 0, { -0.51667f, 166.91667f } },      // Nauru
    { 3381, TZ_LEAF, 0, { -19.01667f, -169.91667f } },    // Niue
    { 3386, TZ_LEAF, 0, { -29.05000f, 167.96667f } },     // Norfolk
    { 3394, TZ_LEAF, 0, { -22.26667f, 166.45000f } },     // Noumea
    { 3401, TZ_LEAF, 0, { -14.26667f, -170.70000f } },    // Pago Pago
    { 3411, TZ_LEAF, 0, { 7.33333f, 134.48333f } },       // Palau
    { 3417, TZ_LEAF, 0, { -25.06667f, -130.08333f } },    // Pitcairn
    { 3426, TZ_LEAF, 0, { 6.96667f, 158.21667f } },       // Pohnpei
    { 3434, TZ_LEAF, 0, { -9.50000f, 147.16667f } },      // Port Moresby
    { 3447, TZ_LEAF, 0, { -21.23333f, -159.76667f } },    // Rarotonga
    { 3457, TZ_LEAF, 0, { 15.20000f, 145.75000f } },      // Saipan
    { 3464, TZ_LEAF, 0, { -17.53333f, -149.56667f } },    // Tahiti
    { 3471, TZ_LEAF, 0, { 1.41667f, 173.00000f } },       // Tarawa
    { 3478, TZ_LEAF, 0, { -21.13333f, -175.20000f } },    // Tongatapu
    { 3488, TZ_LEAF, 0, { 19.28333f, 166.61667f } },      // Wake
    { 3493, TZ_LEAF, 0, { -13.30000f, -176.16667f } },    // Wallis
    { 3500, TZ_LEAF, 0, { -34.60000f, -58.45000f } },     // Buenos Aires
    { 3513, TZ_LEAF, 0, { -28.46667f, -65.78333f } },     // Catamarca
    { 3523, TZ_LEAF, 0, { -31.40000f, -64.18333f } },     // Cordoba
    { 3531, TZ_LEAF, 0, { -24.18333f, -65.30000f } },     // Jujuy
    { 3537, TZ_LEAF, 0, { -29.43333f, -66.85000f } },     // La Rioja
    { 3546, TZ_LEAF, 0, { -32.88333f, -68.81667f } },     // Mendoza
    { 3554, TZ_LEAF, 0, { -51.63333f, -69.21667f } },     // Rio Gallegos
    { 3567, TZ_LEAF, 0, { -24.78333f, -65.41667f } },     // Salta
    { 3573, TZ_LEAF, 0, { -31.53333f, -68.51667f } },     // San Juan
    { 3582, TZ_LEAF, 0, { -33.31667f, -66.35000f } },     // San Luis
    { 3591, TZ_LEAF, 0, { -26.81667f, -65.21667f } },     // Tucuman
    { 3599, TZ_LEAF, 0, { -54.80000f, -68.30000f } },     // Ushuaia
    { 3607, TZ_LEAF, 0, { 39.76833f, -86.15806f } },      // Indianapolis
    { 3620, TZ_LEAF, 0, { 41.29583f, -86.62500f } },      // Knox
    { 3625, TZ_LEAF, 0, { 38.37556f, -86.34472f } },      // Marengo
    { 3633, TZ_LEAF, 0, { 38.49194f, -87.27861f } },      // Petersburg
    { 3644, TZ_LEAF, 0, { 37.95306f, -86.76139f } },      // Tell City
    { 3654, TZ_LEAF, 0, { 38.74778f, -85.06722f } },      // Vevay
    { 3660, TZ_LEAF, 0, { 38.67722f, -87.52861f } },      // Vincennes
    { 3670, TZ_LEAF, 0, { 41.05139f, -86.60306f } },      // Winamac
    { 3678, TZ_LEAF, 0, { 38.25417f, -85.75944f } },      // Louisville
    { 3689, TZ_LEAF, 0, { 36.82972f, -84.84917f } },      // Monticello
    { 3700, TZ_LEAF, 0, { 47.26417f, -101.77778f } },     // Beulah
    { 3707, TZ_LEAF, 0, { 47.11639f, -101.29917f } },     // Center
    { 3714, TZ_LEAF, 0, { 46.84500f, -101.41083f } },     // New Salem
};
