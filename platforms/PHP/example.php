Header("Cache-Control: must-revalidate");
$offset = 10;
$ExpStr = "Expires: " . gmdate("D, d M Y H:i:s", time() - $offset) . " GMT";
Header($ExpStr);
//include ("getid.php");
$host     = "localhost"; // Host name
$username = "####################"; // Mysql username
$password = "####################"; // Mysql password
$db_name  = "####################"; // Database name
$tbl_name = "####################"; // Table name

// Connect to server and select database.
mysql_connect("$host", "$username", "$password")or die("cannot connect");
mysql_select_db("$db_name")or die("cannot select DB");

// Retrieve data from database
$sql    = "SELECT * FROM ############## WHERE ID = '5'";
$result = mysql_query($sql);
?>

// Start looping rows in mysql database. 
while($rows=mysql_fetch_array($result))
{
  ?>LAT  echo $rows['WAYLAT001'];
  ?>LONG echo $rows['WAYLONG001'];
}
mysql_close();
 ?>
